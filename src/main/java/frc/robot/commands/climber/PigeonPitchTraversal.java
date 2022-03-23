package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;

public class PigeonPitchTraversal extends CommandBase{
    private double lastPitch;
    private double curPitch;
    public static double MIN_PITCH_VELOCITY = 1;
    private Timer time;

    public PigeonPitchTraversal(){
        addRequirements(Climber.getInstance());
        time = new Timer();
    }

    public void initialize() {
        lastPitch = Double.MAX_VALUE;
        curPitch = Drivetrain.getInstance().getPigeon().getPitch();
        time.reset();
        time.start();
    }

    public void execute() {
        if(time.hasElapsed(0.1)) {
            lastPitch = curPitch;
            curPitch = Drivetrain.getInstance().getPigeon().getPitch();
            time.reset();
            time.start();
        }

    }

    public boolean isFinished() {
        return false;//(curPitch < 0) && curPitch - lastPitch <= MIN_PITCH_VELOCITY;
    }

    public void end(boolean interrupted) {
        // if(!interrupted)
        //     CommandScheduler.getInstance().schedule(new SetClimberPosition(pullUpSetValue, ClimberManual.MAGNITUDE_BACKWARD));
    }
}
