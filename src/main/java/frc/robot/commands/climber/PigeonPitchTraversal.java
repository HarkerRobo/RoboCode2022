package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;

public class PigeonPitchTraversal extends CommandBase{
    private double lastPitch;
    private double curPitch;
    private double pullUpSetValue;
    public static double MIN_PITCH_VELOCITY = 1;

    public PigeonPitchTraversal(double upValue){
        pullUpSetValue = upValue;
    }

    public void initialize() {
        lastPitch = Double.MAX_VALUE;
        curPitch = Drivetrain.getInstance().getPigeon().getPitch();
    }

    public void execute() {
        lastPitch = curPitch;
        curPitch = Drivetrain.getInstance().getPigeon().getPitch();
    }

    public boolean isFinished() {
        return (curPitch < 0) && curPitch - lastPitch <= MIN_PITCH_VELOCITY;
    }

    public void end(boolean interrupted) {
        if(!interrupted)
            CommandScheduler.getInstance().schedule(new SetClimberPosition(pullUpSetValue, ClimberManual.MAGNITUDE_BACKWARD));
    }
}
