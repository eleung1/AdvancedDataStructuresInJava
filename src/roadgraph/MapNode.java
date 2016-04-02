package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

/**
 * MapNode represents a vertex in MapGraph.
 * 
 * This is either an intersection or dead-end.
 * 
 * @author Eric Leung
 *
 */
public class MapNode {
	
	// latitude, longitude pair for this node
	private GeographicPoint geoPoint;
	
	// Out-going edges of this node.
	private List<MapEdge> edges;
	
	public GeographicPoint getGeoPoint() {
		return geoPoint;
	}

	public void setGeoPoint(GeographicPoint geoPoint) {
		this.geoPoint = geoPoint;
	}

	public List<MapEdge> getEdges() {
		if ( edges == null ) {
			edges = new ArrayList<MapEdge>();
		}
		return edges;
	}

}
