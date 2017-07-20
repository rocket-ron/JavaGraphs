package roadgraph;

import java.util.List;
import java.util.ArrayList;

import geography.GeographicPoint;

/**
 * GraphRoute contains a list of GraphPath objects and acts as a container for segments where each segment is
 * a GraphPath and the ordered list of segments constitutes and overall Route.
 *
 * Created by rcordell on 9/23/16.
 */
public class GraphRoute {

    List<GraphPath> theRoute;

    public GraphRoute() {
        this.theRoute = new ArrayList<>();
    }

    /**
     * Returns the length of this Route object
     * @return the total length of the GraphRoute
     */
    public double getLength() {
        return this.theRoute.stream().mapToDouble(path -> path.getLength()).sum();
    }

    /**
     * Add a GraphPath object as the next item in the list of GraphPaths
     * @param path the GraphPath to addEdge to the route
     */
    public void add(GraphPath path) {
        this.theRoute.add(path);
    }

    public GraphEdge getEnd() {
        if (!this.theRoute.isEmpty()) {
            return this.theRoute.get(this.theRoute.size() - 1).getEnd();
        } else {
            return null;
        }
    }

    public GraphEdge getStart() {
        if (!this.theRoute.isEmpty()) {
            return this.theRoute.get(0).getEnd();
        } else {
            return null;
        }
    }

    /**
     * Return this GraphRoute as a single GraphPath that incorporates all the GraphPath segments into one
     * @return the resulting GraphPath object
     */
    public GraphPath getAsPath() {
        GraphPath result = null;

        for (GraphPath path : this.theRoute) {
            if (result == null) {
                result = GraphPath.copy(path);
            } else {
                result = result.add(path);
            }
        }
        return result;
    }

    /**
     * Return this route as a single list of GeographicPoints
     * @return
     */
    public List<GeographicPoint> getAsGeographicPoints() {
        return this.getAsPath().getGeographicPoints();
    }

}
