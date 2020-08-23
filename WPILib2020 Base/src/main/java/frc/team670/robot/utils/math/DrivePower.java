package frc.team670.robot.utils.math;

/** Class for retrieving DrivePower info */
public class DrivePower {
	private double left;
	private double right;

	public DrivePower(double left, double right) {
		this.left = left;
		this.right = right;
	}

	public double getLeft() {
		return left;
	}

	public double getRight() {
		return right;
	}

}
