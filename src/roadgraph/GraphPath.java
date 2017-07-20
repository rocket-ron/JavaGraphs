package roadgraph;

import geography.GeographicPoint;

import java.util.List;
import java.util.ArrayList;
import java.util.stream.Collectors;

/**
 * GraphPath is a simple object that contains a list of GraphEdge objects and implements the Comparable interface
 *
 * Created by rcordell on 9/23/16.
 */
public class GraphPath implements Comparable<GraphPath> {

    List<GraphEdge> thePath;

    public GraphPath() {
        thePath = new ArrayList<>();
    }

    public GraphPath(int size) {
        thePath = new ArrayList<>(size);
    }

    /**
     * Return the length of this path
     * @return the length of the path
     */
    public double getLength() {
        return thePath.stream().mapToDouble(edge -> edge.getDistance()).sum();

    }

    /**
     * Add a new GraphEdge to the path
     * @param e the GraphEdge object to addEdge to the path
     */
    public void addEdge(GraphEdge e) {
        if (!this.thePath.isEmpty() && e.getTo().getCumulativeDistance() == Double.POSITIVE_INFINITY) {
            e.getTo().setCumulativeDistance(this.getEnd().getTo().getCumulativeDistance() +
                    e.getTo().distance(this.getEnd().getTo()));
        }
        thePath.add(e);
    }

    /**
     * Add a GraphPath object to this instance and return a new combined instance that consists of the
     * GraphEdge objects in this instance followed by the GraphEdge objects in the parameter instance. If
     * the last GraphEdge object of this instance is the same GeographicPoint as the first GraphEdge of the
     * parameter then the duplicate is eliminated. The final result has the cumulativeDistance and
     * remainingDistances of the GraphEdges corrected for the resulting GraphPath return object.
     * @param gp the GraphPath object to add
     * @return a GraphPath object
     */
    public GraphPath add(GraphPath gp) {
        GraphPath result = new GraphPath(this.getPath().size() + gp.getPath().size());

        double cumulativeDistance = 0.0;

        // copy the GraphEdge objects of this GraphPath into the result
        for (GraphEdge edge : this.getPath()) {
            GraphEdge edgeToAdd = GraphEdge.copy(edge);
            if (result.getEnd() != null) {
                cumulativeDistance += result.getEnd().getTo().distance(edge.getTo());
            }
            edgeToAdd.getTo().setCumulativeDistance(cumulativeDistance);

            result.addEdge(GraphEdge.copy(edge));
        }

        boolean firstEdge = true;
        for (GraphEdge edge : gp.getPath()) {
            if (firstEdge && this.getEnd().equals(edge)) {
                firstEdge = false;
                continue;
            } else {
                GraphEdge edgeToAdd = edge.copy(edge);
                cumulativeDistance += result.getEnd().getTo().distance(edgeToAdd.getTo());
                edgeToAdd.getTo().setCumulativeDistance(cumulativeDistance);
                result.addEdge(edgeToAdd);
            }
        }

        for (GraphEdge edge : result.getPath()) {
            edge.getTo().setRemainingDistance(cumulativeDistance - edge.getTo().getCumulativeDistance());
        }

        return result;
    }


    /**
     * Get the List of GraphEdges that make up this path object
     * @return
     */
    public List<GraphEdge> getPath() {
        return thePath;
    }

    /**
     * Get the start of the path
     * @return the GraphEdge at the start of the path
     */
    public GraphEdge getStart() {
        if (!this.thePath.isEmpty()) {
            return thePath.get(0);
        } else {
            return null;
        }
    }

    /**
     * Get the end of the path
     * @return the GraphEdge that represents the end of the path
     */
    public GraphEdge getEnd() {
        if (!this.thePath.isEmpty()) {
            return this.thePath.get(this.thePath.size() - 1);
        } else {
            return null;
        }
    }

    /**
     * Make a copy of a GraphPath object
     * @param p the GraphPath object to copy
     * @return the copy
     */
    public static GraphPath copy(GraphPath p) {
        if (p == null) {
            throw new IllegalArgumentException("Parameter must not be null");
        }
        GraphPath result = new GraphPath();
        for (GraphEdge edge : p.getPath()) {
            result.addEdge(GraphEdge.copy(edge));
        }

        return result;
    }


    /**
     * Get the list of GeographicPoint objects that make up the path
     * @return a List of GeographicPoint objects
     */
    public List<GeographicPoint> getGeographicPoints() {
        return thePath.stream().map(edge -> (GeographicPoint)(edge.getTo())).collect(Collectors.toList());
    }

    /**
     * Compare GraphPath objects by their length
     * @param p the GraphPath object to which we compare this one
     * @return negative, zero, or positive integer based on greater, equal, or less than p
     */
    @Override
    public int compareTo(GraphPath p) {
        return (int)(this.getLength() - p.getLength());
    }

    @Override
    public String toString() {
        String response = "";
        for (GraphEdge edge : this.getPath()) {
            response += edge.toString() + "\n";
        }
        return response;
    }
}
