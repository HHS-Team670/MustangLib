package frc.team670.mustanglib.pathfinder;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.team670.mustanglib.utils.math.sort.Node;

/**
 * Node representing Pose. Inspired by Hemlock 5712
 * @author ethan c ;)
 */
public class PoseNode implements Node<PoseNode> {
	public Pose2d pose;
	private List<PoseNode> neighbors;
	/**
	 * Constructs a new pose node at the given coordinates
	 * @param x the x coordinate
	 * @param y the y coordinate
	 */
	public PoseNode(double x, double y) {
		this.pose = new Pose2d(x, y, Rotation2d.fromDegrees(0));
		this.neighbors = new ArrayList<>();
	}
	/**
	 * constructs a new pose node at given coordinates and rotations
	 * @param x the x coordinate
	 * @param y the y coordinate
	 * @param holonomicRotation the rotation of the node
	 */
	public PoseNode(double x, double y, Rotation2d holonomicRotation) {
		this.pose = new Pose2d(x, y, Rotation2d.fromDegrees(0));
		this.neighbors = new ArrayList<>();
	}
	/**
	 * Constructs a new pose node with the passed in pose
	 * @param currentPose the pose of the new node
	 */
	public PoseNode(Pose2d currentPose) {
		this.pose = currentPose;
		this.neighbors = new ArrayList<>();
	}

	/**
	 * constructs a new pose node at given coordinates and rotations
	 * @param coordinates the x and y position of the node
	 * @param holonomicRotation the rotation of the node
	 */
	public PoseNode(Translation2d coordinates, Rotation2d holonomicRotation) {
		this.pose = new Pose2d(coordinates, holonomicRotation);
		this.neighbors = new ArrayList<>();
	}


	/**
	 *  
	 * @return the x coordinate of this pose
	 */
	public double getX() {
		return pose.getX();
	}
	/**
	 * 
	 * @return the y coordinate of this pose
	 */
	public double getY() {
		return pose.getY();
	}
	/**
	 * 
	 * @return the rotation of this pose
	 */
	public Rotation2d getHolRot() {
		return pose.getRotation();
	}
	
	/**
	 * Adds a neighbor to this node for graph algorithms
	 *@param neighbor the neighbor to add 
	 */
	@Override
	public void addNeighbor(PoseNode neighbor) {
		this.neighbors.add(neighbor);
	}
	/**
	 * @param target The target to calculate distance from
	 * @return double the distance
	 */
	@Override
	public double getHeuristicDistance(PoseNode target) {
		return this.pose.getTranslation().getDistance(target.pose.getTranslation());
	}
	
	/**
	 *@return the neighbors
	 */
	@Override
	public List<PoseNode> getNeighbors() {
		return neighbors;
	}
}
