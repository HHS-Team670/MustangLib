/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj;

import edu.wpi.first.hal.DutyCycleJNI;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

/**
 * Class to read a duty cycle PWM input.
 *
 * <p>PWM input signals are specified with a frequency and a ratio of high to low
 * in that frequency. There are 8 of these in the roboRIO, and they can be
 * attached to any {@link DigitalSource}.
 *
 * <p>These can be combined as the input of an AnalogTrigger to a Counter in order
 * to implement rollover checking.
 *
 */
public class DutyCycle implements Sendable, AutoCloseable {
  // Explicitly package private
  final int m_handle;

  private final DigitalSource m_source;

  /**
   * Constructs a DutyCycle input from a DigitalSource input.
   *
   * <p>This class does not own the inputted source.
   *
   * @param digitalSource The DigitalSource to use.
   */
  public DutyCycle(DigitalSource digitalSource) {
    m_handle = DutyCycleJNI.initialize(digitalSource.getPortHandleForRouting(),
        digitalSource.getAnalogTriggerTypeForRouting());

    m_source = digitalSource;
    int index = getFPGAIndex();
    HAL.report(tResourceType.kResourceType_DutyCycle, index + 1);
    SendableRegistry.addLW(this, "Duty Cycle", index);
  }

  /**
   * Close the DutyCycle and free all resources.
   */
  @Override
  public void close() {
    DutyCycleJNI.free(m_handle);
  }

  /**
   * Get the frequency of the duty cycle signal.
   *
   * @return frequency in Hertz
   */
  public int getFrequency() {
    return DutyCycleJNI.getFrequency(m_handle);
  }

  /**
   * Get the output ratio of the duty cycle signal.
   *
   * <p>0 means always low, 1 means always high.
   *
   * @return output ratio between 0 and 1
   */
  public double getOutput() {
    return DutyCycleJNI.getOutput(m_handle);
  }

  /**
   * Get the raw output ratio of the duty cycle signal.
   *
   * <p>0 means always low, an output equal to getOutputScaleFactor() means always
   * high.
   *
   * @return output ratio in raw units
   */
  public int getOutputRaw() {
    return DutyCycleJNI.getOutputRaw(m_handle);
  }

  /**
   * Get the scale factor of the output.
   *
   * <p>An output equal to this value is always high, and then linearly scales down
   * to 0. Divide the result of getOutputRaw by this in order to get the
   * percentage between 0 and 1.
   *
   * @return the output scale factor
   */
  public int getOutputScaleFactor() {
    return DutyCycleJNI.getOutputScaleFactor(m_handle);
  }

  /**
   * Get the FPGA index for the DutyCycle.
   *
   * @return the FPGA index
   */
  @SuppressWarnings("AbbreviationAsWordInName")
  public final int getFPGAIndex() {
    return DutyCycleJNI.getFPGAIndex(m_handle);
  }

  public int getSourceChannel() {
    return m_source.getChannel();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Duty Cycle");
    builder.addDoubleProperty("Frequency", this::getFrequency, null);
    builder.addDoubleProperty("Output", this::getOutput, null);

  }
}
