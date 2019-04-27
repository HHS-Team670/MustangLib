package frc.team670.robot.utils.search;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;

/**
 * Implementation of breadth first search
 * For details refer to https://www.geeksforgeeks.org/breadth-first-search-or-bfs-for-a-graph/
 */

public class BreadthFirstSearch {

    private static List<Edge> result;

    private BreadthFirstSearch() {}

    public static List<Edge> search(Node start, Node destination) {

        if(start == destination){
            return new ArrayList<Edge>();
        }

        Queue<Edge> queue = new LinkedList<>();
        List<Edge> explored = new ArrayList<>();
        List<Edge> startEdges = Arrays.asList(start.getEdges());

        Map<Edge, Edge> parentNodes = new HashMap<Edge, Edge>();
        queue.addAll(startEdges);

        if(queue.size() < 1) {
            throw new IllegalArgumentException("Invalid input. Make sure it isn't an island!");
        }

        while(!queue.isEmpty()){
            Edge current = queue.remove();
            if(current.getDest() == destination) {
                List<Edge> shortestPath = new ArrayList<Edge>();
                shortestPath.add(current);
                Node node = current.getSource();
                while(node != start) {
                    Edge upPath = parentNodes.get(current);
                    node = upPath.getSource();
                    current = upPath;
                    shortestPath.add(current);
                }
                Collections.reverse(shortestPath);
                return shortestPath;
            }
            else{
                List<Edge> edges = Arrays.asList(current.getDest().getEdges());
                for(Edge edge : edges) {
                    if(!explored.contains(edge)) {
                        parentNodes.put(edge, current);
                        queue.add(edge);
                        explored.add(edge);
                    }
                }
            }
            explored.add(current);
        }

        throw new IllegalArgumentException("Invalid input. Make sure it isn't an island!");

    }

}
