package frc.team670.mustanglib.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class LEDSubsystem extends MustangSubsystemBase {
    private static final int length = 70;
    private AddressableLED led;
    private Status status = Status.BLANK;

    public LEDSubsystem(int port) {
        this.led = new AddressableLED(port);
        led.setLength(length);
    }

    public void setStatus(Status status) {
        this.status = status;
    }
    
    @Override
    public void mustangPeriodic() {
        led.setData(status.buffer);
        led.start();
        
    }

    @Override
    public void debugSubsystem() {
        SmartDashboard.putString("LED Status", status.toString());        
    }

    @Override
    public HealthState checkHealth() {
        return HealthState.GREEN;
    }

    public enum Status {
        SHOOTING(makeBuffer(length, new Color(0, 0, 255))),
		INTAKING(makeBuffer(length, new Color(255,0,0))),
        ONE_BALL(makeBuffer(length, new Color(0,255,0))),
        TWO_BALL(makeBuffer(length, new Color(255,0,255))),
        FLEX(makeBuffer(length, new Color(150,150, 150))),
        BLANK(makeBuffer(length, new Color(0,0,0)));

        public final AddressableLEDBuffer buffer;

        private Status(AddressableLEDBuffer buffer) {
            this.buffer = buffer;
        }

        private static AddressableLEDBuffer makeBuffer(int length, Color color) {
            AddressableLEDBuffer buffer  = new AddressableLEDBuffer(length);

            for(int i=0; i < length; i++) {
                buffer.setRGB(i, (int) color.red, (int) color.green, (int) color.blue);
            }
            return buffer;
        }
    }
}
