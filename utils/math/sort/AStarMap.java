package frc.team670.mustanglib.utils.math.sort;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Implementation of an A* search algorithm https://www.geeksforgeeks.org/a-search-algorithm/
 * 
 * @author ctchen, rghosh670
 */
public class AStarMap {

    protected final List<Node> nodes = new ArrayList<>();
    protected final List<Edge> edges = new ArrayList<>();

    // Add a node to the navigation mesh
    public void addNode(Node node) {
        this.nodes.add(node);
    }

    public int getNodeSize() {
        return nodes.size();
    }

    public Node getNode(int index) {
        return nodes.get(index);
    }

    /**
     * Runs the search function, returning a List ordered in the Edges to take from one start Node
     * to destination
     * 
     * @param start The node that the user wishes to start from
     * @param destination The target node that the user wishes to reach
     * @return A path of nodes that the search algorithm has found.
     * @exception IllegalArgumentException throws if the Node you are starting from has no open
     *            paths from it Returns empty if destination and start are same node
     */
    public List<Node> search(Node start, Node destination) {

        // Use A* search to find the shortest path through the navigation mesh
        Set<Node> closedSet = new HashSet<>();
        Set<Node> openSet = new HashSet<>();
        Map<Node, Double> gScore = new HashMap<>();
        Map<Node, Double> fScore = new HashMap<>();
        Map<Node, Node> cameFrom = new HashMap<>();
        gScore.put(start, 0.0);
        fScore.put(start, start.getHeuristicDistance(destination));
        openSet.add(start);

        while (!openSet.isEmpty()) {

            Node current = getLowestFScore(openSet, fScore);
            if (current.equals(destination)) {
                return getPath(current, cameFrom);
            }
            openSet.remove(current);
            closedSet.add(current);
            for (Node neighbor : current.getNeighbors()) {
                if (!closedSet.contains(neighbor)) {
                    double tentativeGScore =
                            gScore.get(current) + current.getHeuristicDistance(neighbor);
                    if (!openSet.contains(neighbor) || tentativeGScore < gScore.get(neighbor)) {
                        cameFrom.put(neighbor, current);
                        gScore.put(neighbor, tentativeGScore);
                        fScore.put(neighbor,
                                gScore.get(neighbor) + neighbor.getHeuristicDistance(destination));
                        // No check needed, .add is a noop if it contains it. Sets don't allow
                        // duplicates
                        openSet.add(neighbor);
                    }
                }
            }
        }

        // If we get here, then no path was found
        return null;
    }

    // Get the node in the open set with the lowest f score
    private Node getLowestFScore(Set<Node> openSet, Map<Node, Double> fScore) {
        Node lowestFScoreNode = null;
        double lowestFScore = Double.MAX_VALUE;
        for (Node node : openSet) {
            double f = fScore.get(node);
            if (f < lowestFScore) {
                lowestFScore = f;
                lowestFScoreNode = node;
            }
        }
        return lowestFScoreNode;

    }

    /**
     * @return the found path (list of nodes)
     */
    private List<Node> getPath(Node current, Map<Node, Node> cameFrom) {
        List<Node> path = new ArrayList<>();
        path.add(current);
        while (cameFrom.containsKey(current)) {
            current = cameFrom.get(current);
            path.add(current);
        }
        Collections.reverse(path);
        return path;
    }

    // /**
    // * @return the found path (list of edges)
    // */
    // private List<Edge> getPath(Node start, Node destination, HashMap<Node, Edge> cameFrom) {
    // List<Edge> path = new ArrayList<Edge>();
    // Node node = destination;

    // if(cameFrom.isEmpty()) {
    // throw new IllegalArgumentException("Invalid argument, check for island");
    // }
    // while (!node.equals(start)) {
    // Edge e = cameFrom.get(node);
    // path.add(e);
    // node = e.getSource();
    // }

    // Collections.reverse(path);
    // return path;
    // }

}
