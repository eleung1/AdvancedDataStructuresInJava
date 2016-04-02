package roadgraph;

/**
 * MapEdge represents the road segments between intersections(MapNode objects).
 * 
 * MapGraph is a directed graph, so each pair of nodes on a 2-way street
 * will have two edges between them, one in each direction.
 * 
 * At the moment this class is only used by MapGraph which is in the same package.
 * So access level for getters and setters are "protected" because we don't want the public to use them.
 * Only this class, subclasses, and classes within this package can access them.
 * 
 * @author Eric Leung
 *
 */
public class MapEdge {
	private MapNode start;		// Starting point of this road segment
	private MapNode end;		// Ending point of this road segment
	private String roadName;	// Name of this road
	private String roadType;	// Kind of road, e.g. "residential"
	private double length;  	// Length of this road segment, in km
	
	protected MapNode getStart() {
		return start;
	}
	
	protected void setStart(MapNode start) {
		this.start = start;
	}
	
	protected MapNode getEnd() {
		return end;
	}
	
	protected void setEnd(MapNode end) {
		this.end = end;
	}
	
	protected String getRoadName() {
		return roadName;
	}
	
	protected void setRoadName(String roadName) {
		this.roadName = roadName;
	}
	
	protected String getRoadType() {
		return roadType;
	}
	
	protected void setRoadType(String roadType) {
		this.roadType = roadType;
	}
	
	protected double getLength() {
		return length;
	}
	
	protected void setLength(double length) {
		this.length = length;
	}
}
