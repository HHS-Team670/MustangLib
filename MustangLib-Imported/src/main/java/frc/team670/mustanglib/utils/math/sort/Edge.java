package frc.team670.mustanglib.utils.math.sort;

/**
 * Represents an edge that connects nodes. 
 */
public interface Edge extends Comparable<Edge> {
    /**
     * @return the "cost" of travelling over this edge
     */
    public int getCost();
    
    /**
     * @return the source node (where this edge starts)
     */
    public Node getSource();

    /*
     * @return the destination node (where this edge ends)
     */        
    public Node getDest();

}