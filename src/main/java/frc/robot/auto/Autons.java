package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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
        Trajectories.fiveBallAuto.get(0),
        Trajectories.fiveBallAuto.get(1),
        Trajectories.fiveBallAuto.get(2),
        Trajectories.fiveBallAuto.get(3));

    public static final SequentialCommandGroup TWO_BALL_AUTO_STEAL_AND_YEET = new SequentialCommandGroup(
        Trajectories.twoBallAutoStealAndYeet.get(0),
        Trajectories.twoBallAutoStealAndYeet.get(1), 
        Trajectories.twoBallAutoStealAndYeet.get(2));

    public static final SequentialCommandGroup TWO_BALL_AUTO_TOP = new SequentialCommandGroup(
        Trajectories.twoBallAutoTop.get(0));

    public static final SequentialCommandGroup THREE_BALL_AUTO = new SequentialCommandGroup(
        Trajectories.threeBallAuto.get(0),
        Trajectories.threeBallAuto.get(1));

    public static final SequentialCommandGroup TWO_BALL_AUTO = new SequentialCommandGroup(
        Trajectories.twoBallAuto.get(0));

    public static final SequentialCommandGroup TWO_BALL_AUTO_MIDDLE = new SequentialCommandGroup(
        Trajectories.twoBallAutoMiddle.get(0));
}