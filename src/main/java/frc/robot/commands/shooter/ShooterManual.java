package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import harkerrobolib.commands.IndefiniteCommand;

/**
 * Shoots with 80% percent output
 */
public class ShooterManual extends IndefiniteCommand {
    
    public ShooterManual() {
        addRequirements(Shooter.getInstance());
    }

    public void execute() {
        // Shooter.getInstance().setPercentOutput(OI.getInstance().getDriverGamepad().getRightTrigger() * 0.8);
    }

    public void end() {
        Shooter.getInstance().setPercentOutput(0);
    }
}