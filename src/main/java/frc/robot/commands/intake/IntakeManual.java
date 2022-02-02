package frc.robot.commands.intake;

import harkerrobolib.commands.IndefiniteCommand;
import frc.robot.OI;
import frc.robot.subsystems.Intake;

/**
 * Command to intake with a set velocity
 */
public class IntakeManual extends IndefiniteCommand {
    public static final double VELOCITY = 0.3;
    
    public IntakeManual() {
        addRequirements(Intake.getInstance());
    }

    public void execute() {
        if(OI.getInstance().getDriverGamepad().getButtonB().get())
        Intake.getInstance().setVelocity(VELOCITY); 
        else
        Intake.getInstance().setVelocity(0); 

    }
    
    public void end(boolean interrupted) {
        Intake.getInstance().setVelocity(0);
    }
}