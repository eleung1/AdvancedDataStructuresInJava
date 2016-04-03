package roadgraph;

/**
 * Model which contains a MapNode as well as its distance from the start node
 * of the search.
 * 
 * Used for searching cheapest path in MapGraph.
 * 
 * @author Eric Leung
 *
 */
public class MapNodeDistanceModel implements Comparable<MapNodeDistanceModel> {
	
	// The actual MapNode.
	private MapNode node;
	
	// Its distance from start.  Used in Dijkstra search to find cheapest path.
	private double distanceFromStart;

	public MapNodeDistanceModel(MapNode n, double d)
	{
		this.node = n;
		this.distanceFromStart = d;
	}
	
	protected MapNode getNode() {
		return node;
	}

	protected void setNode(MapNode node) {
		this.node = node;
	}
	
	protected double getDistanceFromStart() {
		return distanceFromStart;
	}

	protected void setDistanceFromStart(double distanceFromStart) {
		this.distanceFromStart = distanceFromStart;
	}
	
	/**
	 * This is needed for the ordering in our PriorityQueue when searching for
	 * shortest path.
	 * 
	 */
	@Override
	public int compareTo(MapNodeDistanceModel model) {
		return Double.compare(distanceFromStart, model.getDistanceFromStart());
	}

}
