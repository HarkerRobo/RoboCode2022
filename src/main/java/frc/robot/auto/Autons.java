package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.HSSwerveDriveController;
import frc.robot.commands.intake.IntakeAutonControlForward;

public class Autons {
    public static SequentialCommandGroup autoPath1 = new SequentialCommandGroup(
        new ParallelCommandGroup(
            new HSSwerveDriveController(Trajectories.goToTrench, Rotation2d.fromDegrees(0)),
            new IntakeAutonControlForward(0.5)), 
        new HSSwerveDriveController(Trajectories.returnFromTrench, Rotation2d.fromDegrees(0)));

}
