package frc.robot.commands.intake;

import harkerrobolib.commands.IndefiniteCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.subsystems.Intake;

/**
 * Command to intake with a set velocity
 */
public class IntakeManual extends IndefiniteCommand {    
    private double speed;

    public IntakeManual(double speed) {
        addRequirements(Intake.getInstance());
        this.speed = speed;
    }   

    public void execute() {
        speed = SmartDashboard.getNumber("intake RPS", 0.1);
        if(OI.getInstance().getDriverGamepad().getButtonB().get())
            Intake.getInstance().setVelocity(speed * Intake.MAX_RPS); 
        else
            Intake.getInstance().setPercentOutput(0); 

    }
    
    public void end(boolean interrupted) {
        Intake.getInstance().setPercentOutput(0);
    }
}