package frc.team670.mustanglib.utils.math.sort;

import java.util.List;

/**
* Models a node on a graph. Needed for A* sort 
* https://www.geeksforgeeks.org/a-search-algorithm/ 
*/
public interface Node<T extends Node<T>> {  
    
    /**
     * @return estimated distance to target node
     */
    public double getHeuristicDistance(T target);

    /**
     * @return a list of the node's neighbors
     */
    public List<T> getNeighbors();

    /**
     * 
     * @param neighbor - neighbor to add
     */
    public void addNeighbor(T neighbor);
}