# Supporting Documentation for RoadGraph Package

## Class: MapGraph

Modifications made to MapGraph:

- Implemented the MapGraph constructor to intialize the adjacency list HashMap. The structure chosen to represent the graph 
is a `HashMap<GeographicPoint, List<RoadGraphEdge>` map. The adjacency list was chosen because the graph is sparse. The map
maps each vertex in the graph to its outbound neighbors. The neighbors are `RoadGraphEdge` classes.

- Implemented the `getNumVertices` method. Since a HashMap is used for the map of adjacency lists, this simply returns the
number of entries in the `HashMap`.

- Implemented the `getNumEdges` method. This method counts the unique number of non-directional edges in the adjacency list map.

- Implemented the `getVertices` method. This method returns the keys of the adjacency list map.

- Implemented the `addVertex` method. This method adds a new entry to the adjacency list map if the vertex is not already present 
and initializes the list itself to an empty `ArrayList`

- Implemented the `addEdge` method. This method adds a `RoadGraphEdge` to the adjacency list of a vertex in the adjacency list map.

- Implemented the `bfs` method. This method follows the algorithm as outlined in the course materials.

- Implemented a `toString` method that prints information about the graph.

- Implemented a private `adjacencyString` method that generates the strings used in the `toString` method.

## Class: RoadGraphEdge

This class implements an edge in the graph and is a simple container for the fields required of edges:

- `to` : the destination `GeographicPoint` object
- `distance` : the distance to the `to` coordinate as `double`
- `roadName` : the name of the road segment as `String`
- `roadType` : the type of the road segment as `String`

The class consists only of a constructor and getter methods for the fields.
The constructor validates the parameters and returns an `IllegalArgumentException` that indicates the invalid parameter.
The class is immutable.

## Overall Design Justification

The `MapGraph` class is given and the only method that is complex is the implementation of the `bfs` algorithm. 

A `deque` implementation of the `Queue` was selected in the `bfs` method as the most flexible since in this case concurrency is not an issue 
while the `ArrayDeque` allows for resizing.

A `Stack` is used to generate the path as computed in the `bfs` method and stored in the `parent map`. Each vertex from the 
`parent map` is pushed on the `Stack` as the `parent map` is walked backwards from the goal to the start to construct the actual path.
The `Stack` is then popped iteratively into a `List` that is ultimately returned from the `bfs` method.
