package frc.team670.mustanglib.utils.motorcontroller.defaultconfig;

import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.SparkMAXFactory.Config;

public class NeoDefaults {
    // for velocity control, all we need is frame 1, which has the velocity value

    public Config VELOCITY_CONTROLLED_NEO = new Config();
    VELOCITY_CONTROLLED_NEO.STATUS_FRAME_0_RATE_MS = 5000;
    VELOCITY_CONTROLLED_NEO.STATUS_FRAME_1_RATE_MS = 20;
    VELOCITY_CONTROLLED_NEO.STATUS_FRAME_2_RATE_MS = 5000;

    // for position control, frame 2 - motor position in turns
    public Config POSITION_CONTROLLED_NEO = new Config();
    POSITION_CONTROLLED_NEO.STATUS_FRAME_0_RATE_MS = 5000;
    POSITION_CONTROLLED_NEO.STATUS_FRAME_1_RATE_MS = 5000;
    POSITION_CONTROLLED_NEO.STATUS_FRAME_2_RATE_MS = 20;
    
    // if it's open-loop or a follower, turn everything down
    public Config UNCONTROLLED_NEO = new Config();
    UNCONTROLLED_NEO.STATUS_FRAME_0_RATE_MS = 5000;
    UNCONTROLLED_NEO.STATUS_FRAME_1_RATE_MS = 5000;
    UNCONTROLLED_NEO.STATUS_FRAME_2_RATE_MS = 5000;

    // if it's a leader, same as above but ALSO turn up 0, which has applied output
    // it's what followers use to set themselves
}
