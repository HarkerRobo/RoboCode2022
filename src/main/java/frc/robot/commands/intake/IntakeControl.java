package frc.robot.commands.intake;

import harkerrobolib.util.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Intake;
import harkerrobolib.commands.IndefiniteCommand;

public class IntakeControl extends IndefiniteCommand {
    Command runForward = new IntakeManualPID(0.5, false);
    Command runBackward = new IntakeManualPID(-0.8, true);
    
    public IntakeControl() {
        addRequirements(Intake.getInstance());

    }

    @Override
    public void execute() {
        
        
        double magnitudeRight = 0; // MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightTrigger(), OI.DEADBAND);
        double magnitudeLeft = 0; // MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftTrigger(),OI.DEADBAND);
        if(magnitudeRight==0 && magnitudeLeft ==0){
            magnitudeRight = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightTrigger(),
            OI.DEADBAND);
            magnitudeLeft = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftTrigger(),
            OI.DEADBAND);
        }   

        if(magnitudeRight==0 && magnitudeLeft ==0){
            magnitudeRight = MathUtil.mapJoystickOutput(OI.getInstance().getOperatorGamepad().getRightTrigger(),
            OI.DEADBAND);
            magnitudeLeft = MathUtil.mapJoystickOutput(OI.getInstance().getOperatorGamepad().getLeftTrigger(),
            OI.DEADBAND);
        }       
        
        
                
        double magnitude;

        if (magnitudeLeft > 0 || magnitudeRight > 0) {

            if (magnitudeLeft > magnitudeRight) {
                runBackward.execute();
            } else {
                runForward.execute();
            }
        }else{
            runForward.end(true);
            runBackward.end(true);

        }
    }

}
