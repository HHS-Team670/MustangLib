package frc.team670.mustanglib.pathfinder;

import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.List;
import frc.team670.mustanglib.pathfinder.Obstacle.PolygonDouble;
import frc.team670.mustanglib.utils.math.sort.AStarSearch;

/**
 * Uses Pose Edge and A Star search in a graph traversal fashion to avoid obstacles on the field
 * @author ethan c :D
 */
public class ObstacleAvoidanceAStarMap {

    private final AStarSearch<PoseNode, PoseEdge> searchAlg = new AStarSearch<>();
    private List<PoseNode> contingencyNodes = new ArrayList<>();
    private final List<PoseEdge> edges = new ArrayList<>();
    private final List<Obstacle> obstacles = new ArrayList<>();
    private PoseNode startNode, endNode;

    /**
     * Constructs a new obstacle avoidance map with the given obstacles as well as the starting position and the destination position
     * @param start the start
     * @param destination the destination position
     * @param obstacles obstacles in the field ex: the charge station from the 2023 field
     */
    public ObstacleAvoidanceAStarMap(PoseNode start, PoseNode destination, List<Obstacle> obstacles) {
        this.startNode = start;
        this.endNode = destination;
        addObstacles(obstacles);
    }
    /**
     * Constructs a new obstacle avoidance map with the given obstacles as well as the starting position and the destination positions and a list of contingency nodes
     * @param start the start
     * @param destination the destination position
     * @param obstacles obstacles in the field
     * @param obstacleContingencyNodes a list of fallback nodes for the navigation mesh
     */
    public ObstacleAvoidanceAStarMap(PoseNode start, PoseNode destination, List<Obstacle> obstacles, List<PoseNode> obstacleContingencyNodes) {
        this.startNode = start;
        this.endNode = destination;
        this.contingencyNodes = obstacleContingencyNodes;
        addObstacles(obstacles);
    }

    
    /**
     * Add a node to the navigation mesh
     * @param node
     */
    public void addNode(PoseNode node) {
        this.contingencyNodes.add(node);
    }
    /**
     * Adds a list of nodes to the navigation mesh
     * @param nodes
     */
    public void addNodes(List<PoseNode> nodes) {
        this.contingencyNodes.addAll(nodes);
    }
    /**
     * @return the node size
     */
    public int getNodeSize() {
        return contingencyNodes.size();
    }

    /**
     * 
     * @param index the index of the node you want to return
     * @return a node at the inputted index
     */
    public PoseNode getNode(int index) {
        return contingencyNodes.get(index);
    }
    /**
     *  Finds and returns a path from the start node to the end node while avoiding obstacles
     * @return The path from the start node to the end node
     */
    public List<PoseNode> findPath() {
        List<PoseNode> fullPath = new ArrayList<>();
        if (intersectsObstacles(new PoseEdge(startNode, endNode))) {
            System.out.println("intersects obstacle!");
            loadMap();
            fullPath = searchAlg.search(startNode, endNode);
        } else {
            System.out.println("no intersection");
            fullPath.add(startNode);
            fullPath.add(endNode);
        }
        return fullPath;
    }
    /**
     * adds a list of obstacles to the list of obstacles
     * @param obstacles
     */
    public void addObstacles(List<Obstacle> obstacles) {
        this.obstacles.addAll(obstacles);
    }

    
    /**
     * Add edges to nodes it doesn't intersect obstacles
     */
    private void loadMap() {
        PoseEdge startToContingency;
        for (PoseNode node : contingencyNodes) {
            startToContingency = new PoseEdge(startNode, node);
            if (!intersectsObstacles(startToContingency)) {
                this.edges.add(startToContingency);
                startToContingency.start.addNeighbor(startToContingency.end);
                startToContingency.end.addNeighbor(startToContingency.start);
            }
        }
        PoseEdge contingencyToEnd;
        for (PoseNode node : contingencyNodes) {
            contingencyToEnd = new PoseEdge(node, endNode);
            if (!intersectsObstacles(contingencyToEnd)) {
                this.edges.add(contingencyToEnd);
                contingencyToEnd.start.addNeighbor(contingencyToEnd.end);
                contingencyToEnd.end.addNeighbor(contingencyToEnd.start);
            }
        }
        PoseEdge contTocont;
        for (PoseNode node : contingencyNodes) {
            for (PoseNode other : contingencyNodes) {
                if (node == other) continue;

                contTocont = new PoseEdge(node, other);
                if(!intersectsObstacles(contTocont)) {
                    this.edges.add(contTocont);
                    contTocont.start.addNeighbor(contTocont.end);
                    contTocont.end.addNeighbor(contTocont.start);
                }
            }
        }
    }

    /**
     * Checks if an passed in edge intersects with obstacles
     * @param edge the edge to be checked
     * @return if the edge intersects with any obstacle
     */
    private boolean intersectsObstacles(PoseEdge edge) {
        for (Obstacle obstacle : obstacles) {
            PolygonDouble polygon = obstacle.polygon;
            for (int i = 0; i < polygon.npoints; i++) {
                int j = (i + 1) % polygon.npoints;
                double x1 = polygon.xpoints[i];
                double y1 = polygon.ypoints[i];
                double x2 = polygon.xpoints[j];
                double y2 = polygon.ypoints[j];
                if (Line2D.linesIntersect(x1, y1, x2, y2, edge.start.getX(), edge.start.getY(),
                        edge.end.getX(), edge.end.getY())) {
                    return true; // if line from start to final interesects with any obstacle lines
                }
            }
        }
        return false;
    }
    /**
     * 
     * @return the edges
     */
    public List<PoseEdge> getEdges() {
        return edges;
    }
}