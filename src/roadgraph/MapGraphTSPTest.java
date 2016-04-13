package roadgraph;

import static org.junit.Assert.*;

import java.util.List;

import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import geography.GeographicPoint;
import util.GraphLoader;

public class MapGraphTSPTest 
{
  private static GeographicPoint pointA;
  private static GeographicPoint pointB;
  private static GeographicPoint pointC;
  private static GeographicPoint pointD;
  
  // Map with 4 cities.  All cities are connected.
  private static MapGraphTSP tspMap4Cities;

  @BeforeClass
  public static void oneTimeSetup()
  {
    tspMap4Cities = new MapGraphTSP();
    GraphLoader.loadRoadMap("data/maps/travelling_salesman_4cities.map", tspMap4Cities);
    
    pointA = new GeographicPoint(1,1);
    pointB = new GeographicPoint(1,2);
    pointC = new GeographicPoint(3,-1);
    pointD = new GeographicPoint(6,2);
    
    double ab = pointA.distance(pointB);
    double ac = pointA.distance(pointC);
    double ad = pointA.distance(pointD);
    double bc = pointB.distance(pointC);
    double bd = pointB.distance(pointD);
    double cd = pointC.distance(pointD);
    
    System.out.println("A-B:"+ab);
    System.out.println("A-C:"+ac);
    System.out.println("A-D:"+ad);
    System.out.println("B-C:"+bc);
    System.out.println("B-D:"+bd);
    System.out.println("C-D:"+cd);
  }
  
  /**
   * Test the greedy algorithm.
   */
  @Test
	public void testTSP_4cities_greedy()
	{
    GeographicPoint startCity = new GeographicPoint(1, 1);
    List<MapNode> greedyRoute = tspMap4Cities.getTspRouteGreedy(startCity);
    
    // Expected greedyRoute: A > B > C > D > A (cost: 1550.34)
    assertEquals(greedyRoute.get(0).getGeoPoint(), pointA);
    assertEquals(greedyRoute.get(1).getGeoPoint(), pointB);
    assertEquals(greedyRoute.get(2).getGeoPoint(), pointC);
    assertEquals(greedyRoute.get(3).getGeoPoint(), pointD);
    assertEquals(greedyRoute.get(4).getGeoPoint(), pointA);
	}
  
  /**
   * Test greedy + 2-opt.  Expected it to return a path at least as good as greedy.
   * 
   */
  @Test
  public void testTSP_4cities_greedy_2opt()
  {
    GeographicPoint startCity = new GeographicPoint(1, 1);
    
    List<MapNode> greedyRoute = tspMap4Cities.getTspRouteGreedy(startCity);
    double greedyDistance = tspMap4Cities.getRouteDistance(greedyRoute);
    System.out.println("Greedy distance: " + greedyDistance);
    
    // Apply 2-opt to the greedyRoute
    System.out.println("=== 2opt");
    List<MapNode> twoOptRoute = tspMap4Cities.twoOpt(greedyRoute);
    for ( MapNode n: twoOptRoute)
    {
      System.out.println(n.getGeoPoint());
    }
    double twoOptDistance = tspMap4Cities.getRouteDistance(twoOptRoute);
    System.out.println("2-opt distance: " + twoOptDistance);
    
    assertTrue(twoOptDistance <= greedyDistance);
  }
}
