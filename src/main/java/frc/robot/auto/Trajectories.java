package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import frc.robot.subsystems.Drivetrain;

public class Trajectories {
    static TrajectoryConfig config = new TrajectoryConfig(Drivetrain.AUTO_MAX_SPEED, Drivetrain.AUTO_MAX_SPEED_ACCELERATION)
                                                         .addConstraint(new SwerveDriveKinematicsConstraint(Drivetrain.getInstance().getKinematics(), Drivetrain.AUTO_MAX_SPEED));

    public static Trajectory moveForward = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(3, 0, Rotation2d.fromDegrees(180)), 
                    new Pose2d(0, 0, Rotation2d.fromDegrees(180))),
            config);

    public static Trajectory rightAndForward = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
                    new Pose2d(2, 0, Rotation2d.fromDegrees(0)), 
                    new Pose2d(4, 2, Rotation2d.fromDegrees(90))),
            config);
     public static Trajectory sShaped = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(-90)), 
                    new Pose2d(2, 0, Rotation2d.fromDegrees(90)), 
                    new Pose2d(4, 0, Rotation2d.fromDegrees(-90))),
            config);
        public static Trajectory goToTrench = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(4,0, Rotation2d.fromDegrees(180)),
                        new Pose2d(0, 1, Rotation2d.fromDegrees(90)),
                        new Pose2d(0, 5, Rotation2d.fromDegrees(90))),
                config);

                public static Trajectory returnFromTrench = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(0,5, Rotation2d.fromDegrees(270)),
                        new Pose2d(0, 2, Rotation2d.fromDegrees(270)),
                        new Pose2d(2, 0, Rotation2d.fromDegrees(0))),
                config);
}
