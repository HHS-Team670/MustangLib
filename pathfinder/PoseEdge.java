package frc.team670.mustanglib.pathfinder;

import frc.team670.mustanglib.utils.math.sort.Edge;

/**
 * Edge of two pose nodes. Inspired by Hemlock 5712
 * @author ethan c :P
 */
public class PoseEdge implements Edge<PoseNode> {
    public PoseNode start, end;
    private double cost;

   /**
    * Initializes a PoseEdge with the given star end, and cost
    * @param start the start of the edge
    * @param end the end of the edge
    * @param cost the cost
    */
    public PoseEdge(PoseNode start, PoseNode end, double cost) {
        this.start = start;
        this.end = end;
        this.cost = cost;
    }
    /**
     * Initialized a PoseEdge with the given start and end as well as the cost which is the heuristic distance between the points
     * @param start the tart of the edge
     * @param end the end of th edge
     */
    public PoseEdge(PoseNode start, PoseNode end) {
        this.start = start;
        this.end = end;
        this.cost = start.getHeuristicDistance(end);
    }

    /**
     * @return the cost
     */
    @Override
    public double getCost() {
        return cost;
    }

    /**
     * @return the start
     */
    @Override
    public PoseNode getSource() {
        return start;
    }
    /**
     * @return the endpoint
     */
    @Override
    public PoseNode getDest() {
        return end;
    }
}
