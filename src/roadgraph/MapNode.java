package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

/**
 * MapNode represents a vertex in MapGraph.
 * 
 * This is either an intersection or dead-end.
 * 
 * At the moment this class is only used by MapGraph which is in the same package.
 * So access level for getters and setters are "protected" because we don't want the public to use them.
 * Only this class, subclasses, and classes within this package can access them.
 * 
 * @author Eric Leung
 *
 */
public class MapNode {
	
	// latitude, longitude pair for this node
	private GeographicPoint geoPoint;
	
	// Out-going edges of this node.
	private List<MapEdge> edges;
	
	protected GeographicPoint getGeoPoint() {
		return geoPoint;
	}
	
	/**
	 * Set the GeographicPoint of this MapNode.
	 * 
	 * @param geoPoint GeographicPoint of this MapNode.
	 */
	protected void setGeoPoint(GeographicPoint geoPoint) {
		this.geoPoint = geoPoint;
	}
	
	/**
	 * Get the out-going edges from this MapNode.
	 * 
	 * Protected because we don't want the public to modify this.
	 * 
	 * @return List of MapEdge objects representing out-going edges from this MapNode.
	 */
	protected List<MapEdge> getEdges() {
		if ( edges == null ) {
			edges = new ArrayList<MapEdge>();
		}
		return edges;
	}

}
