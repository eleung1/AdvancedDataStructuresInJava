package roadgraph;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * Course project extension:
 * 
 * Option 2: We've talked through a number of algorithms for the Travelling Salesperson Problem. 
 * Create a new method which takes a set of vertices and returns a route (not necessarily the best) 
 * which visits each vertex and returns back to the start. If you want to go a bit further, 
 * you can try multiple solutions we've discussed (e.g., Greedy vs Greedy with 2-opt) and 
 * compare the length of path returned. Note, you'll need to use your A* or Dijstra Algorithm 
 * to determine distances between vertices. Also, recognize that in real world map data, 
 * the path from one vertex to another might include stopping at another vertex in the set. 
 * You could manage that by ignoring the multiple visits to the node or by adding a way to 
 * mark that node as visited (if you pick that path).
 * 
 * @author Eric Leung
 *
 */
public class MapGraphTSP extends MapGraph
{
  /**
   * Travelling Salesperson Problem.  Given a set of cities, return a 
   * route(not necessarily the best) which will visit each city once 
   * and return back to the start.
   * 
   * The set of cities are all vertices in this MapGraph.
   * 
   * @param start
   * @return
   */
  public List<MapNode> getTspRoute(GeographicPoint start)
  {
    // First get the route using greedy algorithm
    List<MapNode> greedyRoute = getTspRouteGreedy(start);
    
    // Now optimize it using 2-opt.  Should return a route no worse than the origin greedyRoute.
    List<MapNode> optimizedRoute = twoOpt(greedyRoute);
    
    return optimizedRoute;
  }
  
  /**
   * Greedy algorithm for finding a route for the travelling salesperson problem.
   * 
   * @param start The home city.
   * @return A route that starts and end in the given start city which will visit
   *         each city once. 
   */
  public List<MapNode> getTspRouteGreedy(GeographicPoint start)
  {
    LinkedList<MapNode> route = new LinkedList<MapNode>();
    double totalCost = 0.0;
    Set<MapNode> visited = new HashSet<MapNode>();
    //Map<MapNode, MapNode> parent = new HashMap<MapNode, MapNode>();
    
    MapNode startCity = getGeoVertexMap().get(start);
    MapNode currCity = startCity;
    visited.add(startCity);
    route.add(startCity);
    
    while ( currCity != null )
    {
      MapNode nextCity = null;
      double minCost = Double.POSITIVE_INFINITY;
      
      // Examine currCity's neighbours
      for ( MapEdge edge: currCity.getEdges() )
      {
        MapNode neighbour = edge.getEnd();
        double cost = edge.getLength();
        
        if ( !visited.contains(neighbour) && cost < minCost )
        {
          minCost = cost;
          nextCity = neighbour;
        }
      }
      
      if ( nextCity != null )
      {
        // found a next city to visit
        route.add(nextCity);
        visited.add(nextCity);
        totalCost += minCost;
        System.out.println("Cost=" + minCost);
      }
      else
      {
        // cost from last city back to home city
        totalCost += route.getLast().getGeoPoint().distance(startCity.getGeoPoint());
        System.out.println("Cost=" + route.getLast().getGeoPoint().distance(startCity.getGeoPoint()));
        
        // we have visited all cities, go back from last city to the start city.
        route.add(startCity);
      }
      
      // nextCity will be null when we have visited all cities, ending the loop.
      currCity = nextCity;
    }
    
    System.out.println("Total cost: " + totalCost);
    System.out.println("GetRouteDistance: " + getRouteDistance(route));
    
    return route;
  }
  
  /**
   * Helper method to find a travelling salesman's route distance.
   * 
   * @param route The route the Salesman will take.
   * @return Total distance of this route.
   */
  public double getRouteDistance(List<MapNode> route)
  {
    double totalDistance = 0.0;
    
    // start city
    MapNode prevCity = route.get(0);
    
    for ( MapNode currCity: route)
    {
      // add the distance between the this city and the previous city to total
      totalDistance += currCity.getGeoPoint().distance(prevCity.getGeoPoint());
      prevCity = currCity;
    }
    
    return totalDistance;
  }
  
  /**
   * 2-opt algorithm for optimizing the given route.
   * 
   * From wikipedia:
   * Given int j and k:
   * 1. take route[1] to route[j-1] and add them in order to newRoute
   * 2. take route[j] to route[k] and add them in reverse to newRoute
   * 3. take route[k+1] to end and add them in order to newRoute
   * return newRoute
   * 
   * Basically nodes from index j to index k are reversed.
   * j and k cannot be the first/last node in the original route because
   * that is the start city and reversing them would invalidate the route.
   * 
   * @param route The original route
   * @return A route that has some nodes swapped based on the given params i and k.
   */
  public List<MapNode> twoOptSwap(List<MapNode> route, int j, int k)
  {
    List<MapNode> newRoute = new LinkedList<MapNode>();
    
    // Add route[1] to route[j-1] to newRoute
    for ( int i = 0; i < j; i++ )
    {
      newRoute.add(route.get(i));
    }
    
    // Add route[j] to route[k] to newRoute
    for ( int i = k; i >= j; i -- )
    {
      newRoute.add(route.get(i));
    }
    
    // Add route[k+1] to end to newRoute
    for ( int i = k+1; i < route.size(); i ++ )
    {
      newRoute.add(route.get(i));
    }
    
    return newRoute;
  }
  
  /**
   * two-opt algorithm to optimize the greedy route for the travelling salesman.
   * 
   * @param start The start city.
   * @return
   */
  public List<MapNode> twoOpt(List<MapNode> route)
  {
    // Validate the route.  It should have at least 3 cities.
    // For instance: Home > City1 > Home
    if ( route == null || route.size() < 3)
    {
      throw new IllegalArgumentException();
    }
    
    // Initialize shortestRoute to be the given route.
    List<MapNode> shortestRoute = route;
    double shortestDistance = getRouteDistance(shortestRoute);
    
    /* 
     * === 3-node route ===
     * A > B > A no need to swap
     * 
     * === 4-node route ===
     * A > B > C > A
     * A > C > B > A
     * 
     * === 5-node route ===
     * A > B > C > D > A
     * A > C > B > D > A
     * A > D > C > B > A
     */
    for ( int j = 1; j < route.size() - 2; j ++ )
    {
      for ( int k = j+1; k < route.size() - 2; k++ )
      {
        List<MapNode> newRoute = twoOptSwap(route, j, k);
        double newDistance = getRouteDistance(newRoute);
        if ( newDistance < shortestDistance )
        {
          // found a better route
          shortestRoute = newRoute;
          shortestDistance = newDistance;
        }
      }
    }
     
    return shortestRoute;
  }
   
  public static void main(String[] args)
  {
    /* Extension: Travelling salesman problem */
    System.out.println("Travelling Salesman Problem");
    MapGraphTSP tspMap = new MapGraphTSP();
    GraphLoader.loadRoadMap("data/maps/travelling_salesman_3cities.map", tspMap);
    GeographicPoint startCity = new GeographicPoint(1, 1);
    List<MapNode> tspRoute = tspMap.getTspRoute(startCity);
    
    for ( MapNode n : tspRoute )
    {
      System.out.println(n.getGeoPoint());
    }
  }
}
