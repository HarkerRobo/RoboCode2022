package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.AlignWithLimelight;
import frc.robot.commands.hood.HoodManual;
import frc.robot.commands.hood.ZeroHood;
import frc.robot.commands.indexer.IndexerManual;
import frc.robot.commands.indexer.MoveBallsToShooter;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.SetIntakeDown;
import frc.robot.commands.shooter.RevShooter;
import frc.robot.commands.shooter.ShooterManual;

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
    public static final SequentialCommandGroup FIVE_BALL_AUTO = new SequentialCommandGroup(
        new SetIntakeDown(),
        Trajectories.fiveBallAuto.get(0).deadlineWith(new IntakeAndIndex(), new ZeroHood()),
        new RevAndAlign(1),
        new ShootAndIndex(2),
        Trajectories.fiveBallAuto.get(1).deadlineWith(new IntakeAndIndex()),
        Trajectories.fiveBallAuto.get(2).deadlineWith(new IntakeAndIndex()),
        new RevAndAlign(1),
        new ShootAndIndex(2),
        Trajectories.fiveBallAuto.get(3).deadlineWith(new IntakeAndIndex()),
        new RevAndAlign(1),
        new ShootAndIndex(2));
    
    public static final SequentialCommandGroup LINE_AND_TURN = new SequentialCommandGroup(
        new SetIntakeDown(),
        Trajectories.lineAndTurn.get(0).deadlineWith(new IntakeAndIndex(), new ZeroHood()),
        new RevAndAlign(1),
        new ShootAndIndex(2));

    // public static final SequentialCommandGroup TURN = new SequentialCommandGroup(
    //         new SetIntakeDown(),
    //         Trajectories.turn.get(0).deadlineWith(new IntakeAndIndex(), new ZeroHood()),
    //         new RevAndAlign(1),
    //         new ShootAndIndex(2));

    // public static final SequentialCommandGroup TWO_BALL_AUTO_STEAL_AND_YEET = new SequentialCommandGroup(
    //     Trajectories.twoBallAutoStealAndYeet.get(0),
    //     Trajectories.twoBallAutoStealAndYeet.get(1), 
    //     Trajectories.twoBallAutoStealAndYeet.get(2));

    // public static final SequentialCommandGroup TWO_BALL_AUTO_TOP = new SequentialCommandGroup(
    //     new SetIntakeDown(),
    //     Trajectories.twoBallAutoTop.get(0).deadlineWith(new RunIntake()),
    //     new AlignWithLimelight().deadlineWith(new RevShooter()),
    //     new WaitCommand(2).deadlineWith(new ShootWithVelocity(), new MoveBallsToShooter()));

    private static class IntakeAndIndex extends ParallelCommandGroup {
        public IntakeAndIndex() {
            super(new RunIntake(), new IndexerManual());
        }
    }

    private static class RevAndAlign extends ParallelDeadlineGroup {
        public RevAndAlign(double timeout) {
            super(new WaitCommand(timeout), new RevShooter(), new AlignWithLimelight(), new HoodManual());
        }
    }

    private static class ShootAndIndex extends ParallelDeadlineGroup {
        public ShootAndIndex(double timeout) {
            super(new WaitCommand(timeout), new ShooterManual(), new MoveBallsToShooter(), new HoodManual());
        }
    }

    public static final SequentialCommandGroup THREE_BALL_AUTO = new SequentialCommandGroup(  
        new SetIntakeDown(),
        Trajectories.threeBallAuto.get(0).deadlineWith(new IntakeAndIndex(), new ZeroHood()),
        new RevAndAlign(2),//dennis was here
        new ShootAndIndex(2),
        Trajectories.threeBallAuto.get(1).deadlineWith(new RunIntake()),
        new RevAndAlign(2),
        new ShootAndIndex(5));

    public static final SequentialCommandGroup ONE_BALL_AUTO = new SequentialCommandGroup(
        new ZeroHood(),
        new WaitCommand(2).deadlineWith(new HoodManual()),
        new ShooterManual().alongWith(new MoveBallsToShooter())
    );

    // public static final SequentialCommandGroup TWO_BALL_AUTO = new SequentialCommandGroup(
    //     new SetIntakeDown(),
    //     new WaitCommand(2).deadlineWith(new MoveBackward(), new ZeroHood(), new RunIntake()),
    //     new WaitCommand(2).deadlineWith(new HoodManual()),
    //     new ShootWithVelocity().alongWith(new MoveBallsToShooter())
    // );
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