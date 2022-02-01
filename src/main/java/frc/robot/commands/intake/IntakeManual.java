package frc.robot.commands.intake;

import harkerrobolib.commands.IndefiniteCommand;
import frc.robot.subsystems.Intake;

/**
 * Command to intake with a set velocity
 */
public class IntakeManual extends IndefiniteCommand {
    public static final double VELOCITY = 1880;
    
    public IntakeManual() {
        addRequirements(Intake.getInstance());
    }

    public void execute() {
        Intake.getInstance().setVelocity(VELOCITY);   
    }
    
    public void end(boolean interrupted) {
        Intake.getInstance().setVelocity(0);
    }
}