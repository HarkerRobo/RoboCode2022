package frc.robot.commands.intake;

import harkerrobolib.commands.IndefiniteCommand;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.Intake;

/**
 * Command to intake with a set velocity
 */
public class IntakeManual extends IndefiniteCommand {    
    private static final double SPEED = 0.6;
    private Debouncer debouncer;

    public IntakeManual() {
        addRequirements(Intake.getInstance());
        debouncer = new Debouncer(0.2, Debouncer.DebounceType.kBoth);
    }   

    public void execute() {
        if((RobotMap.DEMO_MODE && OI.getInstance().getDriverGamepad().getRawButton(10)) || 
            Math.max(OI.getInstance().getDriverGamepad().getRightTrigger(), OI.getInstance().getOperatorGamepad().getLeftTrigger()) > 0.5) {
            Intake.getInstance().setVelocity(SPEED * Intake.MAX_RPS); 
            Intake.getInstance().state = 1;
        }
        else if((RobotMap.DEMO_MODE && OI.getInstance().getDriverGamepad().getRawButton(9)) || 
            Math.max(OI.getInstance().getDriverGamepad().getLeftTrigger(), OI.getInstance().getOperatorGamepad().getLeftTrigger()) > 0.5) {
            Intake.getInstance().setVelocity(-SPEED * Intake.MAX_RPS); 
            Intake.getInstance().state = -1;
        }
        else {
            Intake.getInstance().setPercentOutput(0);
            Intake.getInstance().state = 0;
        }
        
        if(debouncer.calculate(Intake.getInstance().state == 0))
            Intake.getInstance().getSolenoid().set(Value.kForward);
        else
            Intake.getInstance().getSolenoid().set(Value.kReverse);
    }
    
    public void end(boolean interrupted) {
        Intake.getInstance().setPercentOutput(0);
    }
}