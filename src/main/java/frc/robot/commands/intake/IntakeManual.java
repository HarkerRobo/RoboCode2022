package frc.robot.commands.intake;

import harkerrobolib.commands.IndefiniteCommand;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.OI;
import frc.robot.subsystems.Intake;

/**
 * Command to intake with a set velocity
 */
public class IntakeManual extends IndefiniteCommand {    
    private static final double SPEED = 0.6;

    public IntakeManual() {
        addRequirements(Intake.getInstance());
    }   

    public void execute() {
        if(Math.max(OI.getInstance().getDriverGamepad().getRightTrigger(), OI.getInstance().getOperatorGamepad().getLeftTrigger()) > 0.5) {
            Intake.getInstance().setVelocity(SPEED * Intake.MAX_RPS); 
            Intake.getInstance().state = 1;
            Intake.getInstance().getSolenoid().set(Value.kReverse);
        }
        else if(Math.max(OI.getInstance().getDriverGamepad().getLeftTrigger(), OI.getInstance().getOperatorGamepad().getLeftTrigger()) > 0.5) {
            Intake.getInstance().setVelocity(-SPEED * Intake.MAX_RPS); 
            Intake.getInstance().state = -1;
            Intake.getInstance().getSolenoid().set(Value.kReverse);
        }
        else {
            Intake.getInstance().setPercentOutput(0);
            Intake.getInstance().state = 0;
            Intake.getInstance().getSolenoid().set(Value.kForward);
        }

    }
    
    public void end(boolean interrupted) {
        Intake.getInstance().setPercentOutput(0);
    }
}