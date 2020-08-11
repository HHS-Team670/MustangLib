/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.utils;

/**
 * Stores PID values to make them simple to pass around.
 * @author shaylandias
 */
public class PIDConstantSet {

    public final double P, I, I_ZONE, D, FF;

    public PIDConstantSet(double P, double I, double I_ZONE, double D, double FF) {
        this.P = P;
        this.I = I;
        this.I_ZONE = I_ZONE;
        this.D = D;
        this.FF = FF;
    }

    public PIDConstantSet(double P, double I, double D, double FF) {
        this(P, I, 0, D, FF);
    }

    public PIDConstantSet(double P, double I, double D) {
        this(P, I, 0, D, 0);
    }
}
