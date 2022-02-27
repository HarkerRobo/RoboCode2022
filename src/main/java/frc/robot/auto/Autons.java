package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.AlignWithLimelight;
import frc.robot.commands.indexer.MoveBallsToShooter;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.SetIntakeDown;
import frc.robot.commands.shooter.RevShooter;
import frc.robot.commands.shooter.ShootWithVelocity;

/**
 * Stores and selects an autonomous routine to run.
 * 
 * @author Chirag Kaushik
 * @author Jatin Kohli
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 * @author Angela Jia
 * @since February 13, 2020
 */
public class Autons {
    // public static final SequentialCommandGroup FIVE_BALL_AUTO = new SequentialCommandGroup(
    //     new SetIntakeDown(),
    //     Trajectories.fiveBallAuto.get(0).deadlineWith(new RunIntake()),
    //     new AlignWithLimelight().deadlineWith(new RevShooter()),
    //     new WaitCommand(2).deadlineWith(new ShootWithVelocity(), new MoveBallsToShooter()),
    //     Trajectories.fiveBallAuto.get(1).deadlineWith(new RunIntake()),
    //     Trajectories.fiveBallAuto.get(2).deadlineWith(new RunIntake()),
    //     Trajectories.fiveBallAuto.get(3));

    // public static final SequentialCommandGroup TWO_BALL_AUTO_STEAL_AND_YEET = new SequentialCommandGroup(
    //     Trajectories.twoBallAutoStealAndYeet.get(0),
    //     Trajectories.twoBallAutoStealAndYeet.get(1), 
    //     Trajectories.twoBallAutoStealAndYeet.get(2));

    // public static final SequentialCommandGroup TWO_BALL_AUTO_TOP = new SequentialCommandGroup(
    //     new SetIntakeDown(),
    //     Trajectories.twoBallAutoTop.get(0).deadlineWith(new RunIntake()),
    //     new AlignWithLimelight().deadlineWith(new RevShooter()),
    //     new WaitCommand(2).deadlineWith(new ShootWithVelocity(), new MoveBallsToShooter()));

    public static final SequentialCommandGroup THREE_BALL_AUTO = new SequentialCommandGroup(
        new SetIntakeDown(),
        Trajectories.threeBallAuto.get(0).deadlineWith(new RunIntake()),
        new AlignWithLimelight().deadlineWith(new RevShooter()),
        new WaitCommand(2).deadlineWith(new ShootWithVelocity(), new MoveBallsToShooter()),
        Trajectories.threeBallAuto.get(1).deadlineWith(new RunIntake()),
        new AlignWithLimelight().deadlineWith(new RevShooter()),
        new WaitCommand(5).deadlineWith(new ShootWithVelocity(), new MoveBallsToShooter()));

    // public static final SequentialCommandGroup TWO_BALL_AUTO = new SequentialCommandGroup(
    //     new SetIntakeDown(),
    //     Trajectories.twoBallAuto.get(0).deadlineWith(new RunIntake()),
    //     new AlignWithLimelight().deadlineWith(new RevShooter()),
    //     new WaitCommand(2).deadlineWith(new ShootWithVelocity(), new MoveBallsToShooter()));

    // public static final SequentialCommandGroup TWO_BALL_AUTO_MIDDLE = new SequentialCommandGroup(
    //     new SetIntakeDown(),
    //     Trajectories.twoBallAutoMiddle.get(0).deadlineWith(new RunIntake()),
    //     new AlignWithLimelight().deadlineWith(new RevShooter()),
    //     new WaitCommand(2).deadlineWith(new ShootWithVelocity(), new MoveBallsToShooter()));
}