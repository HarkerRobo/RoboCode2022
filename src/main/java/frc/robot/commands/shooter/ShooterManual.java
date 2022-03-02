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
        
    }

    public void end() {
        Shooter.getInstance().setPercentOutput(0);
    }
}