package roadgraph;

import geography.GeographicPoint;

/**
 * Created by rcordell on 9/19/16.
 *
 * This class extends the GeographicPoint class in order to allow use of GeographicPoints as elements in a PriorityQueue
 * that are sorted based on cumulative distances from a start node in the graph.
 */
public class GraphNode extends GeographicPoint implements Comparable<GraphNode> {

    double cumulativeDistance = 0.0;
    double remainingDistance = 0.0;

    public GraphNode(double latitude, double longitude, double distance) {
        super(latitude, longitude);
        this.cumulativeDistance = distance;
    }

    public GraphNode(GeographicPoint location, double distance) {
        super(location.getX(), location.getY());
        this.cumulativeDistance = distance;
    }

    /**
     * get the cumulative distance used from the start node to this node. The distance should be set either as
     * part of the constructor or in a subsequent call to initialize cumulative distances for the graph
     * @return the Double that represents the cumulative distance from the start node to this node
     */
    public double getCumulativeDistance() {return this.cumulativeDistance; }

    /**
     * set the cumulative distance from the start node to this node
     * @param cumulativeDistance represents the distance from the start node to this node
     */
    public void setCumulativeDistance(double cumulativeDistance) { this.cumulativeDistance = cumulativeDistance; }

    /**
     * get the remaining distance from this GraphNode to the goal
     * @return a double that represents the straighline distance from this node to the goal
     */
    public double getRemainingDistance() {
        return this.remainingDistance;
    }

    /**
     * set the remaining distance from this node to the goal
     * @param goalNode the GraphNode of the goal
     */
    public void setRemainingDistance(GraphNode goalNode) {
        this.remainingDistance = this.distance(goalNode);
    }

    /**
     * toString method override returns a string representation of this object
     * @return
     */
    public String toString() {
        return " [NODE at location (" + super.toString() + "), Cumulative Distance: " +
                getCumulativeDistance() + " Remaining Distance: " + getRemainingDistance() + " ]";
    }

    /**
     * Comparator override compares two GraphNode objects by their cumulative distances and returns a negative,
     * zero, or positive value if this instance of the object has a cumulative distance that is greater, equal, or less
     * than the comparable object cumulative distance, respectively.
     *
     * @param g The GraphNode object to compare
     * @return negative, zero, or positive integer depending on whether the cumulative distance is greater, equal, or less
     * than the cumulative distance of the comparable object.
     */
    @Override
    public int compareTo(GraphNode g) {
        return (int)Math.signum(this.getCumulativeDistance() - g.getCumulativeDistance() +
                this.remainingDistance - g.getRemainingDistance());
    }

}
