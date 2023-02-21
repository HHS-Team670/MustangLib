package frc.team670.mustanglib.utils.math.sort;

/**
* Models a node on a graph. Needed for A* sort 
* https://www.geeksforgeeks.org/a-search-algorithm/ 
*/
public interface Node {  
    
    /**
     * @return estimated distance to target node
     */
    public double getHeuristicDistance(Node target);

    /**
     * @return a list of the node's neighbors
     */
    public Node[] getNeighbors();

    /**
     * 
     * @param neighbor - neighbor to add
     */
    public void addNeighbor(Node neighbor);
}