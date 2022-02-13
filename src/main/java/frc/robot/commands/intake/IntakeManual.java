package frc.robot.commands.intake;

import harkerrobolib.commands.IndefiniteCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.subsystems.Intake;

/**
 * Command to intake with a set velocity
 */
public class IntakeManual extends IndefiniteCommand {    
    private double speed = 0.4;

    public IntakeManual() {
        addRequirements(Intake.getInstance());
    }   

    public void execute() {
        // speed = SmartDashboard.getNumber("intake RPS", 0.1);
        if(OI.getInstance().getDriverGamepad().getRightTrigger() > 0.5) {
            Intake.getInstance().setVelocity(speed * Intake.MAX_RPS); 
            Intake.getInstance().state = 1;
        }
        else if(OI.getInstance().getDriverGamepad().getLeftTrigger() > 0.5) {
            Intake.getInstance().setVelocity(-speed * Intake.MAX_RPS); 
            Intake.getInstance().state = -1;
        }
        else {
            Intake.getInstance().setPercentOutput(0); 
            Intake.getInstance().state = 0;
        }

    }
    
    public void end(boolean interrupted) {
        Intake.getInstance().setPercentOutput(0);
    }
}