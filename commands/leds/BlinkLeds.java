package frc.team670.mustanglib.commands.leds;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.team670.mustanglib.subsystems.LEDSubsystem;
import frc.team670.mustanglib.utils.LEDColor;
import frc.team670.mustanglib.utils.Logger;

public class BlinkLeds extends InstantCommand {
    private LEDSubsystem leds;
    private LEDColor color;

    public BlinkLeds(LEDSubsystem leds, LEDColor c) {
        Logger.consoleError("Blinking LEDS");
        this.leds = leds;
        this.color = c;
    }

    public void initialize() {
        leds.blink(color);
    } 
}
