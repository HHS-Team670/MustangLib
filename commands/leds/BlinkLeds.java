package frc.team670.mustanglib.commands.leds;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.team670.mustanglib.subsystems.LEDSubsystem;
import frc.team670.mustanglib.subsystems.LEDSubsystem.Color;

public class BlinkLeds extends InstantCommand {
    private LEDSubsystem leds;
    private Color color;

    public BlinkLeds(LEDSubsystem leds, Color c) {
        this.leds = leds;
        this.color = c;
    }

    public void initialize() {
        leds.blink(color);
    }
}
