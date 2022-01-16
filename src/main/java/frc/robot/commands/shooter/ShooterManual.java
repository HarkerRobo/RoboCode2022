package frc.robot.commands.shooter;

import frc.robot.OI;
import frc.robot.subsystems.Shooter;
import harkerrobolib.commands.IndefiniteCommand;

public class ShooterManual extends IndefiniteCommand {
    
    public void execute() {
        Shooter.getInstance().setPercentOutput(OI.getInstance().getDriverGamepad().getRightTrigger());
    }

    public void end() {
        Shooter.getInstance().setPercentOutput(0);
    }
}