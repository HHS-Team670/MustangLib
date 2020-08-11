package frc.team670.robot.utils;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.XboxController;

public class MustangController extends XboxController {

    private Notifier rumbler;
    private boolean isRumbling;
    private long targetRumbleTime;

    public enum DPadState {
        NEUTRAl, UP, UP_RIGHT, RIGHT, DOWN_RIGHT, DOWN, DOWN_LEFT, LEFT, UP_LEFT;
    }

    public static class XboxButtons {
        // Controller Buttons
        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int RIGHT_BUMPER = 6;
        public static final int LEFT_BUMPER = 5;
        public static final int BACK = 7;
        public static final int START = 8;
        public static final int LEFT_JOYSTICK_BUTTON = 9;
        public static final int RIGHT_JOYSTICK_BUTTON = 10;

        // Controller Axes
        /**
         * Left = Negative, Right = Positive [-1, 1]
         */
        public static final int LEFT_STICK_X = 0;
        /**
         * Up = Negative, Down = Positive [-1, 1]
         */
        public static final int LEFT_STICK_Y = 1;
        /**
         * Left = Positive, Right = Negative [-1, 1]
         */
        public static final int LEFT_TRIGGER_AXIS = 2;
        /**
         * Pressed = Positive [0, 1]
         */
        public static final int RIGHT_TRIGGER_AXIS = 3;
        /**
         * Left = Negative, Right = Positive [-1, 1]
         */
        public static final int RIGHT_STICK_X = 4;
        /**
         * Up = Negative, Down = Positive [-1, 1]
         */
        public static final int RIGHT_STICK_Y = 5;
    }

    public MustangController(int port) {
        super(port);
        isRumbling = false;
        targetRumbleTime = System.currentTimeMillis() - 10;
        rumbler = new Notifier(new Runnable() {
            public void run() {
                if(isRumbling) {
                    checkRumble();
                }
            }
          });
        rumbler.startPeriodic(0.125);
    }

    // helps you get varoius axis and buttons on the XBox controller
    public double getLeftStickX() {
        return super.getRawAxis(XboxButtons.LEFT_STICK_X);
    }

    public double getLeftStickY() {
        return super.getRawAxis(XboxButtons.LEFT_STICK_Y);
    }

    public double getLeftTriggerAxis() {
        return super.getTriggerAxis(Hand.kLeft);
    }

    public double getRightTriggerAxis() {
        return super.getTriggerAxis(Hand.kRight);
    }

    public double getRightStickX() {
        return super.getRawAxis(XboxButtons.RIGHT_STICK_X);
    }

    public double getRightStickY() {
        return super.getRawAxis(XboxButtons.RIGHT_STICK_Y);
    }

    public boolean getLeftBumper() {
        return super.getBumper(Hand.kLeft);
    }

    public boolean getRightBumper() {
        return super.getBumper(Hand.kRight);
    }

    public boolean getLeftJoystickButton() {
        return super.getStickButton(Hand.kLeft);
    }

    public boolean getRightJoystickButton() {
        return super.getStickButton(Hand.kRight);
    }

    public int getPOVValue() {
        return super.getPOV();
    }

    /**
     * Sets the rumble on the controller
     * 
     * @param power The desired power of the rumble [0, 1]
     * @param time The time to rumble for in seconds
     */
    public void rumble(double power, double time) {
        setRumblePower(power);
        isRumbling = true;
        targetRumbleTime = System.currentTimeMillis() + (long)(time * 1000);
    }

    /**
     * Sets the rumble on the controller
     * 
     * @param power The desired power of the rumble [0, 1]
     * @param time  The time to rumble for in seconds
     */
    private void setRumblePower(double power) {
        setRumble(RumbleType.kLeftRumble, power);
        setRumble(RumbleType.kRightRumble, power);
    }

    private void checkRumble() {
        if(System.currentTimeMillis() >= targetRumbleTime) {
            setRumblePower(0);
            isRumbling = false;
        }
    }

    // gets angle of the DPad on the XBox controller pressed with increments of 45
    // degree angle.
    // returns neutal or -1 when nothing is pressed
    public DPadState getDPadState() {

        int angle = super.getPOV();

        if (angle == 0) {
            return DPadState.UP;
        } else if (angle == 45) {
            return DPadState.UP_RIGHT;
        } else if (angle == 90) {
            return DPadState.RIGHT;
        } else if (angle == 135) {
            return DPadState.DOWN_RIGHT;
        } else if (angle == 180) {
            return DPadState.DOWN;
        } else if (angle == 225) {
            return DPadState.DOWN_LEFT;
        } else if (angle == 270) {
            return DPadState.LEFT;
        } else if (angle == 315) {
            return DPadState.UP_LEFT;
        } else {
            return DPadState.NEUTRAl;
        }

    }

}