package frc.team670.mustanglib.utils;


/**
 * Takes in Joystick inputs and determines if they should be the same (the driver might be holding the sticks slightly
 * at different positions, but means for them to be the same). It then adjusts the motor outputs based on the NavX gyroscope
 * angle to make the robot hold a straight line in this situation.
 */
public class TankJoystickCalculator {

    private double tolerance;
    private double angleToHold; // The angle that the robot should be holding to drive straight

    /**
     * 
     * Creates an instance of the calculator
     * 
     * @param tolerance The difference in Joystick input as a percent that this calculator takes to be equal
     */
    public TankJoystickCalculator(double tolerance) {
        this.tolerance = tolerance;
    }

    /**
     * 
     * Adjust inputs based on if they are within tolerance to follow the desired heading angle
     * 
     * @return a double array holding the adjusted motor outputs on a scale of [-1,1] in an array [leftOutput, rightOutput]
     */
    public double[] adjustInputs(double leftInput, double rightInput){
        double[] outputs = new double[2];
        /* TODO put code in here to determine if the inputs are equivalent using tolerance (remember it is a percent)
         *    Check that either both are positive or both are negative, then check:
         *    Math.abs((leftInput - rightInput)/Math.min(leftInput, rightInput)) < tolerance
         * if the angle should be held, and an angle is not already stored, store the angle
         * adjust the inputs to be equal, then based on error from actual heading angle and desired heading
         * create new outputs, then put them into the outputs array to be used to drive the robot
         */
        return outputs;
    }

}