package frc.team670.mustanglib.utils.math.filter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.WPIUtilJNI;

/**
* A class that limits the rate of change of an input value. Useful for implementing voltage,
* setpoint, and/or output ramps. A slew-rate limit is most appropriate when the quantity being
* controlled is a velocity or a voltage; when controlling a position, consider using a {@link
    * edu.wpi.first.math.trajectory.TrapezoidProfile} instead.
    */
public class SlewRateLimiter {
    private double m_accelRateLimit, m_decelRateLimit;
    private double m_prevVal;
    private double m_prevTime;
    
    /**
    * Creates a new SlewRateLimiter with the given rate limit and initial value.
    *
    * @param accelRateLimit The rate-of-change limit for accleration, in units per second.
    * @param decelRateLimit The rate-of-change limit for decleration, in units per second.
    * @param initialValue The initial value of the input.
    */
    public SlewRateLimiter(double accelRateLimit, double decelRateLimit,double initialValue) {
        m_accelRateLimit = accelRateLimit;
        m_decelRateLimit = decelRateLimit;
        m_prevVal = initialValue;
        m_prevTime = WPIUtilJNI.now() * 1e-6;
    }
    
    /**
    * Creates a new SlewRateLimiter with the given rate limit and an initial value of zero.
    *
    * @param accelRateLimit The rate-of-change limit for accleration, in units per second.
    * @param decelRateLimit The rate-of-change limit for decleration, in units per second.
    */
    public SlewRateLimiter(double accelRateLimit, double decelRateLimit) {
        this(accelRateLimit, decelRateLimit, 0);
    }
    
    /**
    * Filters the input to limit its slew rate.
    *
    * @param input The input value whose slew rate is to be limited.
    * @return The filtered value, which will not change faster than the slew rate.
    */
    public double calculate(double input) {
        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - m_prevTime;

        if(input * m_prevVal > 0) { // accelerating
            m_prevVal += MathUtil.clamp(input - m_prevVal, -m_accelRateLimit * elapsedTime, m_accelRateLimit * elapsedTime);
        } else { // decellerating
            m_prevVal += MathUtil.clamp(input - m_prevVal, -m_decelRateLimit * elapsedTime, m_decelRateLimit * elapsedTime);
        }
        m_prevTime = currentTime;
        return m_prevVal;
    }
    
    /**
    * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
    *
    * @param value The value to reset to.
    */
    public void reset(double value) {
        m_prevVal = value;
        m_prevTime = WPIUtilJNI.now() * 1e-6;
    }

    /**
    * Sets the limits of the current SlewRateLimiter
    *
    * @param accelRateLimit The rate-of-change limit for accleration, in units per second.
    * @param decelRateLimit The rate-of-change limit for decleration, in units per second.
    */
    public void setLimits(double accelRateLimit, double decelRateLimit) {
        this.m_accelRateLimit = accelRateLimit;
        this.m_decelRateLimit = decelRateLimit;
    }
}