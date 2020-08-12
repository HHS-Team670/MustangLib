package frc.team670.mustanglib.dataCollection.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Ultrasonic;

public class DIOUltrasonic {

    private DigitalOutput triggerPin;
    private DigitalInput echoPin;
    private Ultrasonic ultrasonic;
    public static final double ULTRASONIC_ERROR_CODE = 99999;

    /**
     * @param horizontalOffset horizontal offset from the center of the robot on the
     *                         side it is on. Left is negative, right is positive.
     */
    public DIOUltrasonic(int tPin, int ePin) {
        triggerPin = new DigitalOutput(tPin);
        echoPin = new DigitalInput(ePin);

        ultrasonic = new Ultrasonic(triggerPin, echoPin);
    }

    /**
     * Gets the ultrasonic distance in inches adjusted for the angle to target and
     * offset of the ultrasonic from the center of the robot.
     * 
     * @param angle The angle to the target from the robot.
     */
    public double getDistance() {
        double distance = getUnadjustedDistance();
        // Math here to adjust the distance based on requirements
        return distance;
    }

    /**
     * Gets the ultrasonic distance unadjusted for offset and angle to target
     */
    public double getUnadjustedDistance() {
        return ultrasonic.getRangeInches();
    }

    public void setUltrasonicAutomaticMode(boolean automaticMode) {
        ultrasonic.setAutomaticMode(automaticMode);
    }

}