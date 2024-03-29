package frc.team670.mustanglib.dataCollection.sensors;

import com.kauailabs.navx.AHRSProtocol.AHRSUpdateBase;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.ITimestampedDataSubscriber;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Driver for a NavX board. Basically a wrapper for the AHRS class. Much of this
 * was taken from 254's code release.
 */
public class NavX {
    protected class Callback implements ITimestampedDataSubscriber {
        @Override
        public void timestampedDataReceived(long system_timestamp, long sensor_timestamp, AHRSUpdateBase update,
                Object context) {
            synchronized (NavX.this) {
                // This handles the fact that the sensor is inverted from our coordinate
                // conventions.
                if (mLastSensorTimestampMs != K_INVALID_TIMESTAMP && mLastSensorTimestampMs < sensor_timestamp) {
                    mYawRateDegreesPerSecond = -1 * 1000.0 * (-mYawDegrees - update.yaw) // Multiply by -1 because our
                                                                                         // system is opposite of
                                                                                         // Bellarmine's
                            / (double) (sensor_timestamp - mLastSensorTimestampMs);
                }
                mLastSensorTimestampMs = sensor_timestamp;
                mYawDegrees = update.yaw; // This used to be multiplied by -1 to flip it, but our coord system is the
                                          // opposite of Bellarmine's
            }
        }
    }

    protected AHRS mAHRS;

    protected Rotation2d mAngleAdjustment = new Rotation2d();
    protected double mYawDegrees;
    protected double mYawRateDegreesPerSecond;
    protected final long K_INVALID_TIMESTAMP = -1;
    protected long mLastSensorTimestampMs;
    private double offSet;

    public NavX(SerialPort.Port port) {
        mAHRS = new AHRS(port);
        resetState();
        mAHRS.registerCallback(new Callback(), null);
    }

    public NavX(I2C.Port port) {
        mAHRS = new AHRS(port);
        resetState();
        mAHRS.registerCallback(new Callback(), null);
    }

    /**
     * Resets and recalibrates the NavX (yaw will go back to zero and offset
     * cleared). Call this right at the beginning of the match.
     */
    public synchronized void reset() {
        mAHRS.reset();
        resetState();
        offSet = 0;
    }

    /** 
     * Resets to a specific angle
     */
    public synchronized void reset(double angle) {
        reset();
        mAHRS.setAngleAdjustment(angle);
    }

    /**
     * Zeroes the yaw for getYawDouble()
     */
    public synchronized void zeroYaw() {
        setOffSetAngle();
        resetState();
    }

    /**
     * Resets the values of mLastSensorTimestampMs, mYawDegrees, and
     * mYawRateDegreesPerSecond to their initial values.
     */

    private void resetState() {
        mLastSensorTimestampMs = K_INVALID_TIMESTAMP;
        mYawDegrees = 0.0;
        mYawRateDegreesPerSecond = 0.0;
    }

    // public synchronized void setAngleAdjustment(Rotation2d adjustment) {
    // mAngleAdjustment = adjustment;
    // }

    /**
     * @return Field Centric Yaw
     */
    private synchronized double getRawYawDegrees() {
        return mYawDegrees;
    }

    /**
     * @return the yaw with offset taken into account. Offset sets the zero of the gyro
     * to the point where zeroYaw() was last called.
     */
    public synchronized double getYawDouble() {
        double rtrnAngle = getRawYawDegrees() - offSet;
        while (rtrnAngle > 180) {
            rtrnAngle = rtrnAngle - 360; // returns the same angle but in range [-180, 180]
        }
        while (rtrnAngle < -180) {
            rtrnAngle = rtrnAngle + 360;
        }
        return rtrnAngle;
    }

    /**
     * @return the angle value obtained from the mAHRS object relative to the last zero position.
     */
    public synchronized double getAngle() {
        return mAHRS.getAngle();
    }

    /**
     * The rate of change of the NavX angle in degrees per second.
     */
    public synchronized double getYawRateDegreesPerSec() {
        return mYawRateDegreesPerSecond;
    }

    /**
     * The rate of change of the NavX angle in radians per second.
     */
    public synchronized double getYawRateRadiansPerSec() {
        return 180.0 / Math.PI * getYawRateDegreesPerSec();
    }



    /**
     * Sets the offSet to the current yaw angle in degrees.
     */
    private synchronized void setOffSetAngle() {
        offSet = getRawYawDegrees();
    }



    /**
     * Gets the field centric yaw (0 degrees is forward for the robot from its
     * starting position),
     * 
     * @return The Yaw (-180, 180) with -180 and 180 being directly backwards.
     */
    public synchronized double getYawFieldCentric() {
        return getRawYawDegrees();
    }

    /**
     * @return the pitch of the NavX
     */
    public double getPitch() {
        return mAHRS.getPitch();
    }

    /**
     * @return the roll of the NavX
     * 
     */
    public double getRoll() {
        return mAHRS.getRoll();
    }
    /**
    * @return if the magnetometer is calibrated
    */

    public boolean isMagnetometerCalibrated() {
        return mAHRS.isMagnetometerCalibrated();
    }

    /**
     * The function returns the fused heading value from the mAHRS object. Heading represents the sensor's best estimate of current heading
     * 
     * @return the heading that represents the sensor's best estimate of current heading
     */
    public double getFusedHeading() {
        return mAHRS.getFusedHeading();
    }

    /**
     * The function returns whether the AHRS (Attitude and Heading Reference System) is currently
     * calibrating.
     * 
     * @return if this navx is calibrating or not
     */
    public boolean isCalibrating() {
        return mAHRS.isCalibrating();
    }

}
