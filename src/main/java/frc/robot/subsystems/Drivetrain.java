package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Specifies a drivetrain with 4 swerve modules
 */
public class Drivetrain extends SubsystemBase {
    private static Drivetrain drivetrain;

    public static final boolean IS_PIGEON_UP = false;

    public static final double DT_WIDTH = 0.5461; // 0.93345 bumper to bumper
    public static final double DT_LENGTH = 0.5969; // 0.88265

    private SwerveDriveKinematics kinematics;
    private SwerveDriveOdometry odometry;

    private Drivetrain() {
        kinematics = new SwerveDriveKinematics(new Translation2d(DT_LENGTH / 2, DT_WIDTH / 2), new Translation2d(DT_LENGTH / 2, -DT_WIDTH / 2), 
        new Translation2d(-DT_LENGTH / 2, DT_WIDTH / 2), new Translation2d(-DT_LENGTH / 2, -DT_WIDTH / 2));
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), new Pose2d(0, 0, new Rotation2d()));
    }

    /**
     * Set angle to given angle and set velocity to given velocity
     */
    public void setAngleAndDriveVelocity(SwerveModuleState[] states) {
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(states);
        double period = 0.02;

        for(int i = 0; i < 4;i ++) {
            SmartDashboard.putNumber("state speed " + i, states[i].speedMetersPerSecond);
        }

        // manually integrate heading, since odometry requires a gyro
        odometry.update(
            odometry.getPoseMeters().getRotation().plus(new Rotation2d(speeds.omegaRadiansPerSecond * period)),
            states
        );
    }

    public double getHeading(){
        return ((!IS_PIGEON_UP) ? -1 : 1) * odometry.getPoseMeters().getRotation().getDegrees();
    }

    public Rotation2d getHeadingRotation() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public SwerveDriveOdometry getOdometry() {
        return odometry;
    }

    public SwerveDriveKinematics getKinematics(){
        return kinematics;
    }

    public static Drivetrain getInstance() {
        if (drivetrain == null) {
            drivetrain = new Drivetrain();
        }
        return drivetrain;
    }
}
