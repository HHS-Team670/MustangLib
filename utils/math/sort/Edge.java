package frc.team670.mustanglib.utils.math.sort;

/**
 * Represents an edge that connects nodes. 
 */
public interface Edge {
    /**
     * @return the "cost" of travelling over this edge
     */
    public double getCost();
    
    /**
     * @return the source node (where this edge starts)
     */
    public Node getSource();

    /*
     * @return the destination node (where this edge ends)
     */        
    public Node getDest();

}