package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;

public class MoveClimbToNextBar extends ParallelCommandGroup {
    public static double MIN_PITCH_VELOCITY = 0.2;
    public MoveClimbToNextBar() {
        super();
        addCommands(new SequentialCommandGroup(
            new SetClimberPosition(Climber.UP_HEIGHT, 0.5),
            new WaitUntilCommand(()-> Drivetrain.getInstance().getPitchVel() > 0 && 
                Drivetrain.getInstance().getPrevPitchVel() < 0)
            // new SetClimberForward()
        ));
        addCommands(new SequentialCommandGroup(
            new WaitCommand(0.2),
            new SetClimberBackward()
        ));
    }
}

