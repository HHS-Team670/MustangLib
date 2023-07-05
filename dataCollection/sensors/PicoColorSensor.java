package frc.team670.mustanglib.dataCollection.sensors;

import java.nio.charset.StandardCharsets;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.SerialPortJNI;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.Timer;


/**
 * This class represents a Pico color sensor and provides methods to read the raw color values and proximity values from the sensor.
 * The sensor is read on a separate thread to avoid blocking the main thread.
 * The class uses JNI to read the sensor data without allocating memory.
 * The sensor data is stored in private fields and accessed through thread-safe methods.
 * The class also provides a method to close the sensor and stop the reading thread.
 */
public class PicoColorSensor implements AutoCloseable {

  /**
   * The RawColor class represents a color with red, green, blue, and infrared components.
   */
  public static class RawColor {
    
    /**
     * Constructs a RawColor object with the specified red, green, blue, and infrared components.
     * @param r the red component of the color
     * @param g the green component of the color
     * @param b the blue component of the color
     * @param _ir the infrared component of the color
     */
    public RawColor(int r, int g, int b, int _ir) {
      red = r;
      green = g;
      blue = b;
      ir = _ir;
    }
    /**
     * Constructs a RawColor object with default values of 0 for all components.
     */
    public RawColor() {
    }
    public int red;
    public int green;
    public int blue;
    public int ir;
  }


  /**
   * This class represents a single character sequence and provides methods to access its data.
   */
  private static class SingleCharSequence implements CharSequence {

    /**
     * The byte array that holds the character sequence.
     */
    public byte[] data;

    /**
     * Returns the length of the character sequence.
     * @return the length of the character sequence
     */
    @Override
    public int length() {
      return data.length;
    }

    /**
     * Returns the character at the specified index in the sequence.
     * @param index the index of the character to return
     * @return the character at the specified index
     */
    @Override
    public char charAt(int index) {
      return (char)data[index];
    }

    /**
     * Returns a new character sequence that is a subsequence of this sequence.
     * @param start the start index of the subsequence
     * @param end the end index of the subsequence
     * @return a new character sequence that is a subsequence of this sequence
     */
    @Override
    public CharSequence subSequence(int start, int end) {
      return new String(data, start, end, StandardCharsets.UTF_8);
    }

  }


 /**
  * The IntRef class is a helper class that holds an integer value.
  */
  private static class IntRef {
    int value;
  }

  /**
   * The function `parseIntFromIndex` parses an integer value from a character sequence starting from a
   * given index, using a comma as a delimiter, and updates the index of the last comma encountered.
   * 
   * @param charSeq A SingleCharSequence object that represents a sequence of characters.
   * @param readLen The parameter "readLen" represents the length of the character sequence that needs
   * to be parsed.
   * @param lastComma lastComma is an IntRef object that holds the index of the last comma found in the
   * charSeq data. It is passed as a reference so that its value can be updated within the method.
   * @return The method is returning an integer value.
   */
   
  int parseIntFromIndex(SingleCharSequence charSeq, int readLen, IntRef lastComma) {
    int nextComma = 0;
    try {
      nextComma = findNextComma(charSeq.data, readLen, lastComma.value);
      int value = Integer.parseInt(charSeq, lastComma.value + 1, nextComma, 10);
      lastComma.value = nextComma;
      return value;
    } catch (Exception ex) {
      return 0;
    }
  }

  private int findNextComma(byte[] data, int readLen, int lastComma) {
    while (true) {
      if (readLen <= lastComma + 1 ) {
        return readLen;
      }
      lastComma++;
      if (data[lastComma] == ',') {
        break;
      }
    }
    return lastComma;
  }

  private final AtomicBoolean debugPrints = new AtomicBoolean();

  private boolean hasColor0;
  private boolean hasColor1;
  private int prox0;
  private int prox1;
  private final RawColor color0 = new RawColor();
  private final RawColor color1 = new RawColor();
  private double lastReadTime;
  private final ReentrantLock threadLock = new ReentrantLock();
  private final Thread readThread;
  private final AtomicBoolean threadRunning = new AtomicBoolean(true);

  /**
   * The above function reads data from a serial port and updates the values of color and proximity
   * sensors.
   */

  private void threadMain() {
    // Using JNI for a non allocating read
    int port = SerialPortJNI.serialInitializePort((byte)1);
    SerialPortJNI.serialSetBaudRate(port, 115200);
    SerialPortJNI.serialSetDataBits(port, (byte)8);
    SerialPortJNI.serialSetParity(port, (byte)0);
    SerialPortJNI.serialSetStopBits(port, (byte)10);

    SerialPortJNI.serialSetTimeout(port, 1);
    SerialPortJNI.serialEnableTermination(port, '\n');

    HAL.report(tResourceType.kResourceType_SerialPort, 2);

    byte[] buffer = new byte[257];
    SingleCharSequence charSeq = new SingleCharSequence();
    charSeq.data = buffer;
    IntRef lastComma = new IntRef();

    RawColor color0 = new RawColor();
    RawColor color1 = new RawColor();

    while (threadRunning.get()) {
      int read = SerialPortJNI.serialRead(port, buffer, buffer.length - 1);
      if (read <= 0) {
        try {
          threadLock.lock();
          this.hasColor0 = false;
          this.hasColor1 = false;
        } finally {
          threadLock.unlock();
        }
        continue;
      }
      if (!threadRunning.get()) {
        break;
      }

      // Trim trailing newline if exists
      if (buffer[read - 1] == '\n') {
        read--;
      }

      if (read == 0) {
        continue;
      }

      if (debugPrints.get()) {
        System.out.println(new String(buffer, 0, read, StandardCharsets.UTF_8));
      }

      lastComma.value = -1;

      boolean hasColor0 = parseIntFromIndex(charSeq, read, lastComma) != 0;
      boolean hasColor1 = parseIntFromIndex(charSeq, read, lastComma) != 0;
      color0.red = parseIntFromIndex(charSeq, read, lastComma);
      color0.green = parseIntFromIndex(charSeq, read, lastComma);
      color0.blue = parseIntFromIndex(charSeq, read, lastComma);
      color0.ir = parseIntFromIndex(charSeq, read, lastComma);
      int prox0 = parseIntFromIndex(charSeq, read, lastComma);
      color1.red = parseIntFromIndex(charSeq, read, lastComma);
      color1.green = parseIntFromIndex(charSeq, read, lastComma);
      color1.blue = parseIntFromIndex(charSeq, read, lastComma);
      color1.ir = parseIntFromIndex(charSeq, read, lastComma);
   /**
    * The function checks if the distance is within the threshold.
    * 
    * @return The method is returning a boolean value.
    */
      int prox1 = parseIntFromIndex(charSeq, read, lastComma);

      double ts = Timer.getFPGATimestamp();

      try {
        threadLock.lock();
        this.lastReadTime = ts;
        this.hasColor0 = hasColor0;
        this.hasColor1 = hasColor1;
        if (hasColor0) {
          this.color0.red = color0.red;
          this.color0.green = color0.green;
          this.color0.blue = color0.blue;
          this.color0.ir = color0.ir;
          this.prox0 = prox0;
        }
        if (hasColor1) {
          this.color1.red = color1.red;
          this.color1.green = color1.green;
          this.color1.blue = color1.blue;
          this.color1.ir = color1.ir;
          this.prox1 = prox1;
        }
      } finally {
        threadLock.unlock();
      }
    }

    SerialPortJNI.serialClose(port);
  }
/**
 * Constructor for PicoColorSensor class. Initializes a new thread to read sensor data.
 */
  public PicoColorSensor() {
    readThread = new Thread(this::threadMain);
    readThread.setName("PicoColorSensorThread");
    readThread.start();
  }
/**
 * Returns whether the sensor 0 is connected or not.
 * @return boolean value indicating whether sensor 0 is connected or not.
 */
  public boolean isSensor0Connected() {
    try {
      threadLock.lock();
      return hasColor0;
    } finally {
      threadLock.unlock();
    }
  }
/**
 * Returns whether the sensor 1 is connected or not.
 * @return boolean value indicating whether sensor 1 is connected or not.
 */
  public boolean isSensor1Connected() {
    try {
      threadLock.lock();
      return hasColor1;
    } finally {
      threadLock.unlock();
    }
  }
/**
 * Returns the raw color data for sensor 0.
 * @return RawColor object containing the raw color data for sensor 0.
 */
  public RawColor getRawColor0() {
    try {
      threadLock.lock();
      return new RawColor(color0.red, color0.green, color0.blue, color0.ir);
    } finally {
      threadLock.unlock();
    }
  }
/**
 * Populates the provided RawColor object with the raw color data for sensor 0.
 * @param rawColor RawColor object to be populated with the raw color data for sensor 0.
 */
  public void getRawColor0(RawColor rawColor) {
    try {
      threadLock.lock();
      rawColor.red = color0.red;
      rawColor.green = color0.green;
      rawColor.blue = color0.blue;
      rawColor.ir = color0.ir;
    } finally {
      threadLock.unlock();
    }
  }
/**
 * Returns the proximity value for sensor 0.
 * @return integer value representing the proximity value for sensor 0.
 */
  public int getProximity0() {
    try {
      threadLock.lock();
      return prox0;
    } finally {
      threadLock.unlock();
    }
  }
/**
 * Returns the raw color data for sensor 1.
 * @return RawColor object containing the raw color data for sensor 1.
 */
  public RawColor getRawColor1() {
    try {
      threadLock.lock();
      return new RawColor(color1.red, color1.green, color1.blue, color1.ir);
    } finally {
      threadLock.unlock();
    }
  }
/**
 * Populates the provided RawColor object with the raw color data for sensor 1.
 * @param rawColor RawColor object to be populated with the raw color data for sensor 1.
 */
  public void getRawColor1(RawColor rawColor) {
    try {
      threadLock.lock();
      rawColor.red = color1.red;
      rawColor.green = color1.green;
      rawColor.blue = color1.blue;
      rawColor.ir = color1.ir;
    } finally {
      threadLock.unlock();
    }
  }
/**
 * Returns the proximity value for sensor 1.
 * @return integer value representing the proximity value for sensor 1.
 */
  public int getProximity1() {
    try {
      threadLock.lock();
      return prox1;
    } finally {
      threadLock.unlock();
    }
  }
/**
 * Returns the timestamp of the last sensor read in seconds.
 * @return double value representing the timestamp of the last sensor read in seconds.
 */
  public double getLastReadTimestampSeconds() {
    try {
      threadLock.lock();
      return lastReadTime;
    } finally {
      threadLock.unlock();
    }
  }
/**
 * Sets whether debug prints should be enabled or not.
 * @param debug boolean value indicating whether debug prints should be enabled or not.
 */
  void setDebugPrints(boolean debug) {
    debugPrints.set(debug);
  }
  /**
 * Closes the PicoColorSensor object and joins the read thread.
 * @throws Exception if an error occurs while closing the PicoColorSensor object.
 */
  @Override
  public void close() throws Exception {
    threadRunning.set(false);
    readThread.join();
  }
}