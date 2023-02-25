package frc.team670.mustanglib.utils.math.sort;

/**
 * Represents an edge that connects nodes. 
 * @author ethan c xd
 */
public interface Edge<T> {
    /**
     * @return the "cost" of travelling over this edge
     */
    public double getCost();
    
    /**
     * @return the source node (where this edge starts)
     */
    public T getSource();

    /*
     * @return the destination node (where this edge ends)
     */        
    public T getDest();

}