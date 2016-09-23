package roadgraph;

import geography.GeographicPoint;

/**
 * Created by rcordell on 9/11/16.
 *
 * The GraphEdge class composes the GraphNode object and is used to implement a graph adjacency list. This
 * class provides extra fields for road name and type as well as distance from the parent node in the adjacency list.
 * It extends GraphNode instead of GeographicPoint so that the node information is in a form that is readily
 * used in a PriorityQueue.
 */
public class GraphEdge {

    private GraphNode to = null;
    private String roadName = null;
    private String roadType = null;
    private double distance = 0.0;

    public GraphEdge(GeographicPoint to, String roadName, String roadType, double distance)
            throws IllegalArgumentException {

        this(new GraphNode(to, 0.0), roadName, roadType, distance);
    }

    public GraphEdge(GeographicPoint to, String roadName, String roadType,
                     double distance, double cumulativeDistance) {
        this(new GraphNode(to, 0.0), roadName, roadType, distance);
        this.to.setCumulativeDistance(cumulativeDistance);
    }

    public GraphEdge(GraphNode to, String roadName, String roadType, double distance) {
        if (to == null) {
            throw new IllegalArgumentException("Parameter 'to' must not be null");
        }
        if (distance < 0) {
            throw new IllegalArgumentException("Parameter 'distance' must be a positive real number");
        }

        this.roadName = roadName;
        this.roadType = roadType;
        this.distance = distance;
        this.to = to;
    }

    public GraphNode getTo() {
        return to;
    }

    public String getRoadName() {
        return roadName;
    }

    public String getRoadType() {
        return roadType;
    }

    public double getDistance() {
        return distance;
    }

    @Override
    public String toString() {
        return "["+ this.to + " => " + this.roadName + " <" + this.roadType + "> " + distance + "]";
    }

    @Override
    public boolean equals(Object o) {
        if (o instanceof GraphEdge && ((GraphEdge)o).getTo().equals(this.getTo()) &&
                ((GraphEdge)o).getRoadName().equals(this.getRoadName()) &&
                ((GraphEdge)o).getRoadType().equals(this.getRoadType()) &&
                ((GraphEdge)o).getDistance() == this.getDistance()) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public int hashCode() {
        return this.getTo().hashCode() +
                this.getRoadName().hashCode() +
                this.getRoadType().hashCode() +
                (int)this.getDistance() % 71;
    }
}
