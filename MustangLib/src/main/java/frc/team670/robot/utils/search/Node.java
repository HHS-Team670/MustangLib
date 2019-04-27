package frc.team670.robot.utils.search;

/**
* Models a node on a graph
*/

public interface Node extends Comparable<Node> {  
    
    /**
     * @return a list of the edges that hit this node
     */
    public Edge[] getEdges();
    
    /**
     * @return estimated distance to target node
     */
    public int getHeuristicDistance(Node target);

}