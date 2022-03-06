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

    
    public static final List<HSSwerveDriveController> threeBallAuto = getDrivetrainCommands(generateDirectTrajectories(
        new Translation2d[] {
            new Translation2d(7.70, 1.89), 
            new Translation2d(7.70, 0.86), 
            new Translation2d(5.76, 1.86)}), Rotation2d.fromDegrees(-90), 
        List.of(Rotation2d.fromDegrees(-90),Rotation2d.fromDegrees(166)));

    public static final List<HSSwerveDriveController> twoBallAuto = getDrivetrainCommands(generateDirectTrajectories(
        new Translation2d[] {
            new Translation2d(7.64, 1.92),
            new Translation2d(7.66, 0.92)}), Rotation2d.fromDegrees(-90), 
        List.of(Rotation2d.fromDegrees(-90)));

     public static final List<HSSwerveDriveController> twoBallAutoMiddle = getDrivetrainCommands(generateDirectTrajectories(
        new Translation2d[] {
            new Translation2d(6.72, 2.51),
            new Translation2d(5.72, 2.03)}), Rotation2d.fromDegrees(47), 
        List.of(Rotation2d.fromDegrees(18)));

    public static final List<HSSwerveDriveController> twoBallAutoTop = getDrivetrainCommands(generateDirectTrajectories(
        new Translation2d[] {
            new Translation2d(6.22, 5.2),
            new Translation2d(5.47, 5.98)}), Rotation2d.fromDegrees(-46),
        List.of(Rotation2d.fromDegrees(-46)));
    
    public static final List<HSSwerveDriveController> twoBallAutoStealAndYeet = getDrivetrainCommands(generateDirectTrajectories(
        new Translation2d[] {
            new Translation2d(6.22, 5.2),
            new Translation2d(5.47, 5.98),
            new Translation2d(5.63, 6.8),
            new Translation2d(4.63, 3.89)}), Rotation2d.fromDegrees(-46),
        List.of(Rotation2d.fromDegrees(-46), Rotation2d.fromDegrees(-132), Rotation2d.fromDegrees(-99)));

    public static final List<HSSwerveDriveController> fiveBallAuto = getDrivetrainCommands(generateDirectTrajectories(
        new Translation2d[] {
            new Translation2d(7.64, 1.92),
            new Translation2d(7.66, 0.92),
            new Translation2d(5.76, 1.86),
            new Translation2d(1.58, 1.47),
            new Translation2d(4.11, 2.96)}), Rotation2d.fromDegrees(-90),
        List.of(Rotation2d.fromDegrees(-90),Rotation2d.fromDegrees(166),Rotation2d.fromDegrees(-140),Rotation2d.fromDegrees(40)));

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