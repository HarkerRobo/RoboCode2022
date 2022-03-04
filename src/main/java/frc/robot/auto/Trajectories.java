package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.commands.drivetrain.HSSwerveDriveController;
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
    public static TrajectoryConfig config = new TrajectoryConfig(HSSwerveDriveController.MAX_DRIVE_VELOCITY,
            HSSwerveDriveController.MAX_DRIVE_ACCELERATION).setKinematics(Drivetrain.getInstance().getKinematics());
    
    public static final Trajectory moveForward = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(0,0,Rotation2d.fromDegrees(90)),
        new Pose2d(0,2,Rotation2d.fromDegrees(90))
    ), config);

    public static final Trajectory moveBackward = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(0,0,Rotation2d.fromDegrees(270)),
        new Pose2d(0,-1.5,Rotation2d.fromDegrees(270))
    ), config);

    public static final Trajectory moveRight = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(0,0,Rotation2d.fromDegrees(0)),
        new Pose2d(2,0,Rotation2d.fromDegrees(0))
    ), config);

    public static final List<Trajectory> moveLeft = generateDirectTrajectories(new Translation2d[] {
        new Translation2d(0,0), new Translation2d(0,2)
    });

    public static final List<HSSwerveDriveController> threepoints = getDrivetrainCommands(generateDirectTrajectories(new Translation2d[] {
        new Translation2d(0,0), new Translation2d(0,-2), new Translation2d(2,-2)}), Rotation2d.fromDegrees(0),
        List.of(Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(270)));

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

    // public static final List<HSSwerveDriveController> twoBallAuto = getDrivetrainCommands(generateDirectTrajectories(new Translation2d[] {
    //     new Translation2d(1.92, -7.64), new Translation2d(0.92, -7.66)}), null, null);
    
    // public static final List<HSSwerveDriveController> twoBallAutoMiddle = getDrivetrainCommands(generateDirectTrajectories(new Translation2d[] {
    //     new Translation2d(2.51, -6.72), new Translation2d(2.03, -5.72)}), null, null);
    
    public static final List<HSSwerveDriveController> threeBallAuto = getDrivetrainCommands(generateDirectTrajectories(
        new Translation2d[] {
            new Translation2d(7.70, 1.89), 
            new Translation2d(7.70, 0.96), 
            new Translation2d(5.76, 1.86)}), 
        Rotation2d.fromDegrees(270), 
        List.of(
            Rotation2d.fromDegrees(270),
            Rotation2d.fromDegrees(166)));

    public static final List<HSSwerveDriveController> twoBallAuto = getDrivetrainCommands(generateDirectTrajectories(new Translation2d[] {
        new Translation2d(1.89, -7.70), new Translation2d(0.96, -7.70)}), Rotation2d.fromDegrees(180), 
        List.of(Rotation2d.fromDegrees(180)));

    // public static final List<HSSwerveDriveController> twoBallAutoTop = getDrivetrainCommands(generateDirectTrajectories(new Translation2d[] {
    //     new Translation2d(5.18, -6.22), new Translation2d(5.81, -5.49)}), null, null);
    
    // public static final List<HSSwerveDriveController> twoBallAutoStealAndYeet = getDrivetrainCommands(generateDirectTrajectories(new Translation2d[] {
    //     new Translation2d(5.13, -6.21), new Translation2d(5.77, -5.51), new Translation2d(6.80, -5.63), new Translation2d(3.89, -4.63)}),
    //     null, null);

    // public static final List<HSSwerveDriveController> fiveBallAuto = getDrivetrainCommands(generateDirectTrajectories(new Translation2d[] {
    //     new Translation2d(1.83,-7.86), new Translation2d(0.90,-7.76), new Translation2d(2.27,-5.70), new Translation2d(1.55,-2.69), new Translation2d(3.10,-3.82)}),
    //     Rotation2d.fromDegrees(180), List.of(Rotation2d.fromDegrees(180),Rotation2d.fromDegrees(242),Rotation2d.fromDegrees(311),Rotation2d.fromDegrees(311)));

    public static List<Trajectory> generateDirectTrajectories(Translation2d[] input){
        List<Trajectory> out = new ArrayList<Trajectory>();
        for(int i=0;i<input.length-1;i++) {
            out.add(TrajectoryGenerator.generateTrajectory(List.of(
                new Pose2d(input[i].getX(),input[i].getY(),getYaw(input[i+1].minus(input[i]))),
                new Pose2d(input[i+1].getX(),input[i+1].getY(),getYaw(input[i+1].minus(input[i])))
            ), config));
        }
        return out;
    }

    public static Rotation2d getYaw(Translation2d subtracted) {
        double angle = Math.toDegrees(Math.atan2(subtracted.getY(), subtracted.getX()));
        if (angle < 0) angle += 360;
        return Rotation2d.fromDegrees(angle);
    }

    public static List<HSSwerveDriveController> getDrivetrainCommands(List<Trajectory> translations, Rotation2d initHeading, 
        List<Rotation2d> headings) {
        List<HSSwerveDriveController> drivetrainCommands = new ArrayList<>();
        drivetrainCommands.add(new HSSwerveDriveController(translations.get(0), initHeading, headings.get(0), true));
        for(int i = 1; i < translations.size(); i++){
            drivetrainCommands.add(new HSSwerveDriveController(translations.get(i), headings.get(i)));
        }
        return drivetrainCommands;
    }
}