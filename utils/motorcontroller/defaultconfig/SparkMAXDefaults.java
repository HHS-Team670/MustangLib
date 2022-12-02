package frc.team670.mustanglib.utils.motorcontroller.defaultconfig;

import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.SparkMAXFactory.Config;

public class SparkMAXDefaults {
    // Status frame info: https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
    // Frame 0 has applied output (used to set followers) and a few status bits
    // Frame 1 has velocity and a few debugging numbers, like voltage and temp
    // Frame 2 has position in turns
    // Frame 3 (can change interval, but not read?) has analog sensor info
    // Frame 4 (unexposed?) has info about alternate encoders

    private static final int SLOW = 5000;
    private static final int FAST = 20;

    // For velocity control, the only data we need is the velocity.
    // For SparkMax, that's frame 1, which has the motor's velocity.
    public Config VELOCITY_CONTROLLED_NEO = new Config();
    VELOCITY_CONTROLLED_NEO.STATUS_FRAME_0_RATE_MS = SLOW;
    VELOCITY_CONTROLLED_NEO.STATUS_FRAME_1_RATE_MS = FAST;
    VELOCITY_CONTROLLED_NEO.STATUS_FRAME_2_RATE_MS = SLOW;

    // for position control, the position is all we need.
    // On SparkMax, frame 2 has the position in turns.
    public Config POSITION_CONTROLLED_NEO = new Config();
    POSITION_CONTROLLED_NEO.STATUS_FRAME_0_RATE_MS = SLOW;
    POSITION_CONTROLLED_NEO.STATUS_FRAME_1_RATE_MS = SLOW;
    POSITION_CONTROLLED_NEO.STATUS_FRAME_2_RATE_MS = FAST;
    
    // If it's being controlled open-loop or is a follower, we don't need any data.
    // We'll turn down every frame we can to minimize usage.
    public Config UNCONTROLLED_NEO = new Config();
    UNCONTROLLED_NEO.STATUS_FRAME_0_RATE_MS = SLOW;
    UNCONTROLLED_NEO.STATUS_FRAME_1_RATE_MS = SLOW;
    UNCONTROLLED_NEO.STATUS_FRAME_2_RATE_MS = SLOW;

    // if it's a leader, same as above but ALSO turn up 0, which has applied output
    // it's what followers use to set themselves
}
