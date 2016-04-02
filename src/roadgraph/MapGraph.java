/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	
	// Map of intersections/dead-ends.  
	// Using Map for efficient lookup by GeographicPoint.
	private Map<GeographicPoint, MapNode> vertices;
	
	// Road segments
	private List<MapEdge> edges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		// Initialization
		vertices = new HashMap<GeographicPoint, MapNode>();
		edges = new ArrayList<MapEdge>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return vertices.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		Set<GeographicPoint> geoPoints = new HashSet<GeographicPoint>();
		
		// Loop through vertices in this graph and put their GeographicPoints
		// into the result set geoPoints.
		for ( GeographicPoint gp : vertices.keySet())
		{
			geoPoints.add(gp);
		}
		
		return geoPoints;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		
		return edges.size();
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
		boolean isSuccess = false;
		
		if ( location != null && !vertices.containsKey(location) )
		{
			// location is not null and not in the graph yet
			
			// Create new node
			MapNode vertex = new MapNode();
			vertex.setGeoPoint(location);
			
			// Put new node into graph
			vertices.put(location, vertex);
			
			isSuccess = true;
		}
		
		return isSuccess;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 2
		if ( from == null || to == null || roadName == null || roadType == null || length < 0 
			|| !isMember(from) || !isMember(to) )
		{
			throw new IllegalArgumentException();
		}
		
		// Get the from and to MapNode objects
		MapNode start = vertices.get(from);
		MapNode end = vertices.get(to);
		
		// Create new edge
		MapEdge edge = new MapEdge();
		edge.setStart(start);
		edge.setEnd(end);
		edge.setRoadName(roadName);
		edge.setRoadType(roadType);
		edge.setLength(length);
		
		// Add this edge to the start node.  Because this is an edge from "start" to "end.
		start.getEdges().add(edge);
		
		// Also add this edge to this graph's edge list to make counting edges a bit easier.
		edges.add(edge);
	}
	
	/**
	 * Helper method to determine if the provided GeographicPoint 
	 * has already been added to the graph.
	 * 
	 * @param point The target Geographic point.
	 * @return True if point is in graph.  False otherwise.
	 */
	private boolean isMember(GeographicPoint point)
	{
		boolean isMember = false;
		
		if ( point != null && vertices.containsKey(point) )
		{
			isMember = true;
		}
		
		return isMember;
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 2
		
		// start and goal should be nodes in the graph.  Otherwise there will be no path between them.
		if ( !isMember(start) || !isMember(goal) )
		{
			throw new IllegalArgumentException();
		}
		
		// Start our path search with the provided start point
		MapNode startNode = vertices.get(start);
		
		// Parent map: for reconstructing the path from start to goal later
		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		
		// Visited nodes 
		// Using GeographicPoings because it has equals() and hashCode() implemented to test
		// membership in the Set.
		Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
		
		// Neighbours to be visited.  Queue(FIFO) is what we want for bfs.
		Queue<MapNode> toExplore = new LinkedList<MapNode>();
		
		toExplore.add(startNode);
		visited.add(start);
		
		boolean found = false;
		while ( !toExplore.isEmpty() )
		{
			// remove head of this queue.
			MapNode currNode = toExplore.remove();
			
			// Hook for visualization.  See writeup.
			nodeSearched.accept(currNode.getGeoPoint());
			
			if ( goal.equals(currNode.getGeoPoint() ) )
			{
				found = true;
				break;
			}
			
			List<MapEdge> currEdges = currNode.getEdges();
			for (MapEdge edge: currEdges)
			{
				// edge.getStart is the currNode, edge.getEnd is the neighbour
				MapNode neighbour = edge.getEnd();
				
				// Explore nodes that we have not visited yet
				if ( !visited.contains(neighbour.getGeoPoint()) )
				{
					toExplore.add(neighbour);
					visited.add(neighbour.getGeoPoint());
					
					// Adding the parent relationship into parentMap.
					parentMap.put(neighbour.getGeoPoint(), currNode.getGeoPoint());
				}
			}
		}
		
		// Initialize resulting path to null
		List<GeographicPoint> path = null;
		
		if ( found )
		{
			// Construct the path from start to goal
			path = constructPath(start, goal, parentMap);
		}
		
		return path;
	}
	
	/**
	 * Helper method to construct the path from start to goal using the given parentMap.
	 * 
	 * This method is currently called by bfs if a path from start to goal is found.
	 * 
	 * @param goal The GeographicPoint representing the goal.
	 * @param parentMap Map containing parent child relationships representing the path from start to goal.
	 * @return List of GeographicPoint objects representing the path from start to goal.
	 */
	private List<GeographicPoint> constructPath(GeographicPoint start, GeographicPoint goal, 
			Map<GeographicPoint, GeographicPoint> parentMap)
	{
		// Construct the path from start to goal
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
			
		// Start with goal and work ourway back to the start using parentMap.
		GeographicPoint currPoint = goal;
		
		while ( !currPoint.equals(start) )
		{
			path.addFirst(currPoint);
			
			// backtrack to its parent
			currPoint = parentMap.get(currPoint);
		}
		// First element is the provided start point
		path.addFirst(start);
		
		return path;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		GeographicPoint start = new GeographicPoint(6, 6);
		GeographicPoint end = new GeographicPoint(0, 0);
		List<GeographicPoint> result = theMap.bfs(start, end);
		for (GeographicPoint p: result)
		{
			System.out.println(p);
		}
		
		/* Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
