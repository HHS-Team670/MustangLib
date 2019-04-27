package frc.team670.robot.utils.search;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;


/**
 * Implementation of depth first search
 * For detail refer to https://www.geeksforgeeks.org/depth-first-search-or-dfs-for-a-graph/
 */
public class DepthFirstSearch {

    private DepthFirstSearch() {

    }

    public static List<Edge> search(Node start, Node destination) {
        return search(start, destination, new TreeSet<Node>());
    }
    private static List<Edge> search(Node start, Node destination, Set<Node> visited) {
        if (start == destination) {
            return new ArrayList<Edge>();
        }
        if (visited.contains(start)) {
            throw new IllegalArgumentException("Invalid input, check for island");
        }
        visited.add(start);

        for (Edge e : start.getEdges()) {
            try {
                List<Edge> subresult = search(e.getDest(), destination, visited);
                List<Edge> result = new ArrayList<Edge>();
                result.add(e);
                result.addAll(subresult);
                return result;
            } catch (IllegalArgumentException ex) {
                // this path doesn't work
            }
        }
        throw new IllegalArgumentException("Invalid input, check for island");
    }
}