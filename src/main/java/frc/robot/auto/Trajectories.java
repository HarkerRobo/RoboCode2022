package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.Drivetrain;

/**
 * Contains all Trajectories used for various autonomous configurations.
 * 
 * @author Jatin Kohli
 * @author Shahzeb Lakhani
 * @author Ada Praun-Petrovic
 * @author Chirag Kaushik
 * @author Angela Jia
 * @author Anirudh Kotamraju
 * @since February 12, 2020
 */
public class Trajectories {
    public static TrajectoryConfig config = new TrajectoryConfig(Drivetrain.MP_MAX_DRIVE_VELOCITY,
            Drivetrain.MP_MAX_DRIVE_ACCELERATION).setKinematics(Drivetrain.getInstance().getKinematics());
    
    public static final Trajectory moveForward = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(0,0,Rotation2d.fromDegrees(90)),
        new Pose2d(0,2,Rotation2d.fromDegrees(90))
    ), config);

    public static final Trajectory moveBackward = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(0,0,Rotation2d.fromDegrees(270)),
        new Pose2d(0,-2,Rotation2d.fromDegrees(270))
    ), config);

    public static final Trajectory moveRight = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(0,0,Rotation2d.fromDegrees(0)),
        new Pose2d(2,0,Rotation2d.fromDegrees(0))
    ), config);

    public static final List<Trajectory> moveLeft = generateDirectTrajectories(new Translation2d[] {
        new Translation2d(0,0), new Translation2d(0,2)
    });

    public static final Trajectory clockwisecircle = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(8.3,1.5,Rotation2d.fromDegrees(180)),
        new Pose2d(8.3-2.438,1.5+2.438,Rotation2d.fromDegrees(90)),
        new Pose2d(8.3, 1.5+4.876,Rotation2d.fromDegrees(0)),
        new Pose2d(8.3+2.438,1.5+2.438,Rotation2d.fromDegrees(270)),
        new Pose2d(8.3,1.5,Rotation2d.fromDegrees(180))
    ), config);

    public static final Trajectory counterclockwisecircle = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(8.3,1.5,Rotation2d.fromDegrees(0)),
        new Pose2d(8.3+2.438,1.5+2.438,Rotation2d.fromDegrees(90)),
        new Pose2d(8.3, 1.5+4.876,Rotation2d.fromDegrees(180)),
        new Pose2d(8.3-2.438,1.5+2.438,Rotation2d.fromDegrees(270)),
        new Pose2d(8.3,1.5,Rotation2d.fromDegrees(180))
    ), config);

    public static final List<Trajectory> twoBallAuto = generateDirectTrajectories(new Translation2d[] {
        new Translation2d(7.64, 1.92), new Translation2d(7.66, 0.92)});
    
    public static final List<Trajectory> twoBallAutoMiddle = generateDirectTrajectories(new Translation2d[] {
        new Translation2d(6.72, 2.51), new Translation2d(5.72, 2.03)});
    
    public static final List<Trajectory> threeBallAuto = generateDirectTrajectories(new Translation2d[] {
        new Translation2d(7.70, 1.89), new Translation2d(7.70, 0.96), new Translation2d(5.76, 1.86)});

    public static final List<Trajectory> twoBallAutoTop = generateDirectTrajectories(new Translation2d[] {
        new Translation2d(6.22, 5.18), new Translation2d(5.49, 5.81)});
    
    public static final List<Trajectory> twoBallAutoStealAndYeet = generateDirectTrajectories(new Translation2d[] {
        new Translation2d(6.21, 5.13), new Translation2d(5.51, 5.77), new Translation2d(5.63, 6.80), new Translation2d(4.63, 3.89)});

    public static final List<Trajectory> fiveBallAuto = generateDirectTrajectories(new Translation2d[] {
        new Translation2d(1.83,-7.86), new Translation2d(0.90,-7.76), new Translation2d(2.27,-5.70), new Translation2d(1.55,-2.69), new Translation2d(3.10,-3.82)});

    public static List<Trajectory> generateDirectTrajectories(Translation2d[] input){
        List<Trajectory> out = new ArrayList<Trajectory>();
        out.add(TrajectoryGenerator.generateTrajectory(List.of(
            new Pose2d(input[0].getX(),input[0].getY(),getAngle(input[1].minus(input[0]))),
            new Pose2d(input[1].getX(),input[1].getY(),getAngle(input[1].minus(input[0])))
        ), config));
        for(int i=1;i<input.length-1;i++) {
            out.add(TrajectoryGenerator.generateTrajectory(List.of(
                new Pose2d(input[i].getX(),input[i].getY(),getAngle(input[i+1].minus(input[i]))),
                new Pose2d(input[i+1].getX(),input[i+1].getY(),getAngle(input[i+1].minus(input[i])))
            ), config));
        }
        return out;
    }

    public static Rotation2d getAngle(Translation2d subtracted) {
        double angle = Math.toDegrees(Math.atan2(subtracted.getY(), subtracted.getX()));
        angle = (angle < 0) ? angle + 360 : angle;
        return Rotation2d.fromDegrees(angle);
    }
}