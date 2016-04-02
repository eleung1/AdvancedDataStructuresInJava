package roadgraph;

/**
 * MapEdge represents the road segments between intersections(MapNode objects).
 * 
 * MapGraph is a directed graph, so each pair of nodes on a 2-way street
 * will have two edges between them, one in each direction.
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
	
	public MapNode getStart() {
		return start;
	}
	
	public void setStart(MapNode start) {
		this.start = start;
	}
	
	public MapNode getEnd() {
		return end;
	}
	
	public void setEnd(MapNode end) {
		this.end = end;
	}
	
	public String getRoadName() {
		return roadName;
	}
	
	public void setRoadName(String roadName) {
		this.roadName = roadName;
	}
	
	public String getRoadType() {
		return roadType;
	}
	
	public void setRoadType(String roadType) {
		this.roadType = roadType;
	}
	
	public double getLength() {
		return length;
	}
	
	public void setLength(double length) {
		this.length = length;
	}
}
