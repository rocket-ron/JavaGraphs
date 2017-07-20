/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.List;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.Stack;
import java.util.HashMap;
import java.util.HashSet;
import java.util.function.Consumer;
import java.util.stream.Collectors;

import geography.GeographicPoint;
import util.GraphLoader;

import java.util.Comparator;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {

	HashMap<GraphNode, List<GraphEdge>> adjList = null;

	enum SearchType { DIJKSTRA, ASTAR, GREEDY, TWO_OPT }
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		adjList = new HashMap<>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return adjList.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		Set<GeographicPoint> vertices =
				this.adjList.keySet().stream()
									 .map(x -> (GeographicPoint)x).collect(Collectors.toSet());

		return vertices;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		HashSet<GraphEdge> edges = new HashSet<>();
		for (GraphNode v : adjList.keySet()) {
			for (GraphEdge edge : adjList.get(v)) {
				if (!edges.contains(edge)) {
					edges.add(edge);
				}
			}
		}
		return edges.size();
	}

	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		GraphNode node = new GraphNode(location, Double.POSITIVE_INFINITY);
		if (adjList != null && !adjList.containsKey(node)) {
			adjList.put(node, new ArrayList<>());
			return true;
		}
		return false;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		GraphNode f = new GraphNode(from, Double.POSITIVE_INFINITY);
		GraphNode t = new GraphNode(to, Double.POSITIVE_INFINITY);

		if (!adjList.containsKey(f) || !adjList.containsKey(t)) {
			throw new IllegalArgumentException("'from' or 'to' parameter is not a known location - perhaps you forgot to addEdge it?");
		}

		GraphEdge edge = new GraphEdge(to, roadName, roadType, length, Double.POSITIVE_INFINITY);

		List<GraphEdge> vertexList = adjList.get(f);

		if (!vertexList.contains(edge)) {
			vertexList.add(edge);
		}
	}

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// initialize our queue, parent map and visited set
		ArrayDeque<GraphNode> queue = new ArrayDeque<>();
		HashMap<GraphNode, GraphNode> parentMap = new HashMap<>();
		HashSet<GraphNode> visited = new HashSet<>();

		GraphNode s = new GraphNode(start, 0.0);
		GraphNode g = new GraphNode(goal, 0.0);

		// enqueue the start value and place in the visited set
		queue.add(s);
		visited.add(s);

		// now do the bfs
		while (!queue.isEmpty()) {

			// dequeue the next node to explore
			GraphNode v = queue.remove();

			// send the notification that we're exploring this node
			nodeSearched.accept(v);

			// if we're at the goal then construct the path from the parent map
			if (v.equals(g)) {
				return getPathFromParentMap(s, g, parentMap);
			}

			// we're not at the goal so get the neighbors of the current node (the list from the adjacency map)
			for (GraphEdge neighbor : adjList.get(v)) {
				if (!visited.contains(neighbor.getTo())) {
					visited.add(neighbor.getTo());
					parentMap.put(neighbor.getTo(), v);
					queue.add(neighbor.getTo());
				}
			}
		}
		return null;
	}

	/**
	 * Given a HashMap of parent-child nodes, construct the path from the start to the goal through the Map
	 * We can construct the path using a stack and tracing the child to parent from the goal to the start. The
	 * goal is pushed on the stack first and then each parent until we get to the start, which is pushed on the stack
	 * last. Then the stack is converted to a List to preserve the order of LIFO.
	 * @param start GraphNode start location
	 * @param goal GraphNode goal location
	 * @param parentMap Map of parent nodes to child nodes
	 * @return a List of GeographicPoint that represents the path, beginning from the start node to the goal node
	 */
	private List<GeographicPoint> getPathFromParentMap(GraphNode start, GraphNode goal,
													   HashMap<GraphNode, GraphNode> parentMap) {
		// use a stack to create the path by walking the map from goal to start
		Stack<GraphNode> path = new Stack<>();
		GraphNode previous = goal;
		while (!previous.equals(start)) {
			path.push(previous);
			previous = parentMap.get(previous);
		}

		// push the start onto the top of the stack
		path.push(start);

		// convert the stack to a List and return
		List<GeographicPoint> result = new ArrayList<>(path.size());
		while (!path.isEmpty()) {
			result.add(path.pop());
		}
		return result;
	}

	/**
	 * Construct the path of GeographicPoint objects by walking the HashMap of parent -> child objects
	 * @param start the start location as a GraphEdge object
	 * @param goal the goal location as a GraphEdge object
	 * @param parentMap the parent map of GraphEdge -> GraphEdge objects
	 * @return the list of GeographicPoints that make up the path from start to goal
	 */
	private GraphPath getPathFromParentMap(GraphEdge start, GraphEdge goal,
													   HashMap<GraphEdge, GraphEdge> parentMap) {
		// use a stack to create the path by walking the map from goal to start
		Stack<GraphEdge> path = new Stack<>();
		GraphEdge previous = goal;
		while (!previous.getTo().equals(start.getTo())) {
			path.push(previous);
			previous = parentMap.get(previous);
		}

		// push the start onto the top of the stack
		path.push(start);

		// convert the stack to a List and return
		GraphPath result = new GraphPath(path.size());
		while (!path.isEmpty()) {
			result.addEdge(path.pop());
		}
		return result;
	}


	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		return search(start,  goal, nodeSearched, SearchType.DIJKSTRA, null).getGeographicPoints();
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		return search(start, goal, nodeSearched, SearchType.ASTAR, null).getGeographicPoints();

	}

	/**
	 * Search algorithm implementation for Dijkstra and AStar search
	 * @param goal end of path
	 * @param start start of path
	 * @param nodeSearched callback for showing current search node
	 * @param searchType Dijkstra or AStar search specification
	 * @param avoidLocations a list of GraphNodes that must not be in the final path
	 * @return a List of GeographicPoints that represent the path from start to goal
	 */
	private GraphPath search(GeographicPoint start, GeographicPoint goal,
							 Consumer<GeographicPoint> nodeSearched, SearchType searchType,
							 Set<GraphEdge> avoidLocations) {
		int counter = 0;
		PriorityQueue<GraphEdge> priorityQueue = new PriorityQueue<>(adjList.size());

		// Pre-populate the visited set with those locations we have to avoid in this calculation
		HashSet<GraphEdge> visited = new HashSet<>(avoidLocations);
		HashMap<GraphEdge, GraphEdge> parentMap = new HashMap<>();

		GraphEdge startNode = new GraphEdge(start, "Start", "Start", 0.0);
		GraphEdge goalNode = new GraphEdge(goal, "End", "End", 0.0);

		startNode.getTo().setCumulativeDistance(0.0);

		// Our graph nodes in the adjacency list have already been initialized so that cumulative distance is
		// Double.POSITIVE_INFINITY so there's no need to initialize that here.

		// enqueue the start edge with cumulative distance of 0
		priorityQueue.add(startNode);

		while (!priorityQueue.isEmpty()) {
			GraphEdge edge = priorityQueue.remove();

			if (!visited.contains(edge)) {
				counter += 1;
				// System.out.println("DIJKSTRA visiting " + node);
				nodeSearched.accept(edge.getTo());
				visited.add(edge);
				if (edge.getTo().equals(goalNode.getTo())) {
					System.out.println("Nodes visited in search: " + counter);
					return getPathFromParentMap(startNode, goalNode, parentMap);
				}
				for (GraphEdge neighbor : adjList.get(edge.getTo())) {
					if (!visited.contains(neighbor)) {
						neighbor.getTo().setCumulativeDistance(edge.getTo().getCumulativeDistance() + neighbor.getDistance());
						if (searchType == SearchType.ASTAR) {
							neighbor.getTo().setRemainingDistance(goalNode.getTo());
						}
						if (!parentMap.containsKey(neighbor)) {
							parentMap.put(neighbor, edge);
						} else {
							for (GraphEdge key : parentMap.keySet()) {
								if (key.equals(neighbor) && key.getTo().compareTo(neighbor.getTo()) > 0) {
									parentMap.remove(key);
									parentMap.put(neighbor, edge);
									break;
								}
							}
						}
						priorityQueue.add(neighbor);
					}
				}
			}
		}
		return null;
	}

	/**
	 * Implement a round trip given a start point and a List of locations. Each location will be visited exactly once
	 * and the resulting path computed based on the search type of Greedy or 2-Opt.
	 * @param locations a List of GeographicPoint objects that specify locations to visit
	 * @param start the start point
	 * @param searchType either Greedy or Greedy with 2-Opt
	 * @return the path as a List of GeographicPoints that begins and ends with the start point
	 * @throws IllegalArgumentException if the start location is included in the List of locations or any null parameter
	 */
	public List<GeographicPoint> roundTrip(List<GeographicPoint> locations, GeographicPoint start,
										   SearchType searchType) {

		if (locations == null || start == null || searchType == null) {
			throw new IllegalArgumentException("Parameters to the roundTrip() method may not be null");
		}
		if (searchType != SearchType.GREEDY && searchType != SearchType.TWO_OPT) {
			throw new IllegalArgumentException("Search type must be GREEDY or TWO_OPT");
		}
		if (locations.contains(start)) {
			throw new IllegalArgumentException("The location list must not contain the start location");
		}

		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {};

		// Create a GraphRoute object to contain our GraphPath segments
		GraphRoute route = new GraphRoute();

		// Make a copy of the location list as a list of GraphNodes that we will use to track nodes we haven't visited.
		List<GraphNode> checkList = locations.stream()
				.map(l -> new GraphNode(l, 0.0))
				.collect(Collectors.toList());

		// Create the first Edge object from the start GeographicPoint
		GraphEdge startNode = new GraphEdge(start, "Start", "Start", 0.0, 0.0);

		// Create the queue used to process the next location along the path
		ArrayDeque<GraphEdge> queue = new ArrayDeque<>();

		// Create a Visited Set and pre-allocated enough space for all nodes in the location list plus the start node
		HashSet<GraphEdge> visited = new HashSet<>(locations.size() + 2);

		// Push the start GraphEdge onto the queue to kick things off
		queue.add(startNode);

		while (!checkList.isEmpty()) {
			// first we need to find the shortest path from the start node to each of the nodes in the location list
			// that will be our first path segment
			GraphPath shortestPath = java.util.Collections.min(checkList.stream()
					.map(node -> search(queue.peekLast().getTo(), node, temp, SearchType.ASTAR, visited))
					.collect(Collectors.toList()));

			// if we didn't find a path then give up
			if (shortestPath == null) { return null; }

			// select the shortest path
			route.add(shortestPath);
			visited.add(shortestPath.getStart());
			queue.add(shortestPath.getEnd());

			// remove the end of the path from the checklist
			checkList.remove(shortestPath.getEnd().getTo());
		}
		visited.remove(startNode);
		GraphPath pathHome = search(route.getEnd().getTo(), start, temp, SearchType.ASTAR, visited);
		if (pathHome == null) {
			return null;
		} else {
			route.add(pathHome);
		}

		// return route.getPointsOnRoute();

		return route.getAsGeographicPoints();
	}

	private String adjacencyString() {
		String s = "(size " + getNumVertices() + "locations +" + getNumEdges() + " roads):\n";

		for (GeographicPoint v : adjList.keySet()) {
			s += "\t"+v+":\n";
			for (GraphEdge e : adjList.get(v)) {
				s += "\t\t" + e+"\n";
			}
		}
		return s;
	}

	/** Return a String representation of the graph
	 * @return A string representation of the graph
	 */
	public String toString() {
		String s = "\nGraph with " + getNumVertices() + " vertices and " + getNumEdges() + " edges.\n";
		if (getNumVertices() <= 20) s += adjacencyString();
		return s;
	}
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.

		/*
		System.out.print(firstMap);


		GeographicPoint testStart1 = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd1 = new GeographicPoint(8.0, -1.0);

		List<GeographicPoint> myTestRoute = firstMap.bfs(testStart1, testEnd1);
		System.out.println(myTestRoute);

		System.out.println("\n-------------------------------- TEST 2 -----------------------------");

		*/

		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */



		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 2 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		System.out.println("\n-------------------------------- TEST 3 -----------------------------");
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 3 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);

		System.out.println("\n-------------------------------- TEST 4 -----------------------------");
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 4 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);

		*/
		
		/* Use this code in Week 3 End of Week Quiz */

		/*
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		*/

		/* This test is for the loop visit problem (traveling salesman) */
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simplelooptest.map", simpleTestMap);

		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		List<GeographicPoint> pointsToVisit = new ArrayList<>();
		pointsToVisit.add(new GeographicPoint(8.0, 3.0));
		pointsToVisit.add(new GeographicPoint(6.5, 0.0));


		System.out.println("--------------------------- Test Loop Path --------------------------\n");
		List<GeographicPoint> testroute = simpleTestMap.roundTrip(pointsToVisit, testStart, SearchType.GREEDY);
		System.out.println(testroute);
	}
	
}

/**
 * Alternative GraphEdge comparator class that uses the distance field to compare two GraphEdges
 */
class GraphEdgeDistanceComparator implements Comparator<GraphEdge> {

	public int compare(GraphEdge edge1, GraphEdge edge2) {
		return (int)(edge1.getDistance() - edge2.getDistance());
	}
}