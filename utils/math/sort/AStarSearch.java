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
 * @author ctchen, rghosh670, ethan c :)
 */
public class AStarSearch<N extends Node<N>, E extends Edge<N>> {

    /**
     * Runs the search function, returning a List ordered in the Edges to take from one start N to
     * destination
     * 
     * @param start The node that the user wishes to start from
     * @param destination The target node that the user wishes to reach
     * @return A path of nodes that the search algorithm has found.
     * @exception IllegalArgumentException throws if the N you are starting from has no open paths
     *            from it Returns empty if destination and start are same node
     */
    public List<N> search(N start, N destination) {

        // Use A* search to find the shortest path through the navigation mesh
        Set<N> closedSet = new HashSet<>();
        Set<N> openSet = new HashSet<>();
        Map<N, Double> gScore = new HashMap<>();
        Map<N, Double> fScore = new HashMap<>();
        Map<N, N> cameFrom = new HashMap<>();
        gScore.put(start, 0.0);
        fScore.put(start, start.getHeuristicDistance(destination));
        openSet.add(start);

        while (!openSet.isEmpty()) {

            N current = getLowestFScore(openSet, fScore);
            if (current.equals(destination)) {
                return getPath(current, cameFrom);
            }
            openSet.remove(current);
            closedSet.add(current);
            for (N neighbor : current.getNeighbors()) {
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
    private N getLowestFScore(Set<N> openSet, Map<N, Double> fScore) {
        N lowestFScoreNode = null;
        double lowestFScore = Double.MAX_VALUE;
        for (N node : openSet) {
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
    private List<N> getPath(N current, Map<N, N> cameFrom) {
        List<N> path = new ArrayList<>();
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
