package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.subsystems.Drivetrain;

public class HSSwerveDriveController extends SwerveControllerCommand {
    public static final double X_KP = 8;
    public static final double X_KI = 0;
    public static final double X_KD = 0;

    public static final double Y_KP = 8;
    public static final double Y_KI = 0;
    public static final double Y_KD = 0;

    public static final double THETA_KP = 4;
    public static final double THETA_KI = 0;
    public static final double THETA_KD = 7;

    public static final double MAX_DRIVE_VELOCITY = 3;
    public static final double MAX_DRIVE_ACCELERATION = 2;

    public static final double MAX_ANGLE_VELOCITY = Math.PI;
    public static final double MAX_ANGLE_ACCELERATION = Math.PI;

    private static PIDController xController = new PIDController(X_KP, X_KI, X_KD);
    private static PIDController yController = new PIDController(Y_KP, Y_KI, Y_KD);
    private static ProfiledPIDController thetaController = 
        new ProfiledPIDController(THETA_KP, THETA_KI, THETA_KD, new Constraints(MAX_ANGLE_VELOCITY, MAX_ANGLE_ACCELERATION));
    private static final double TURN_TIME = 0.6;
    
    private Rotation2d initHeading;
    Trajectory trajectory;
    private double startTime;
    private boolean isFirst;

  
    public HSSwerveDriveController(Trajectory trajectory, Rotation2d initHeading, Rotation2d finalHeading, boolean isFirst) {
        super(trajectory, Drivetrain.getInstance().getOdometry()::getPoseMeters, 
                Drivetrain.getInstance().getKinematics(), xController, yController, thetaController,() -> finalHeading, 
                Drivetrain.getInstance()::setAngleAndDriveVelocity, Drivetrain.getInstance());
        this.trajectory = trajectory;
        this.initHeading = initHeading;
        this.isFirst = isFirst;
    }

    public HSSwerveDriveController(Trajectory trajectory, Rotation2d finalHeading) {
        this(trajectory, null, finalHeading, false);
    }
    
    @Override
    public void initialize() {
        super.initialize();
        if(isFirst && initHeading != null) {
            Drivetrain.getInstance().getPigeon().setYaw(initHeading.getDegrees());
            Drivetrain.getInstance().getOdometry().resetPosition(new Pose2d(trajectory.getInitialPose().getTranslation(), Drivetrain.getInstance().getHeadingRotation()),
            Drivetrain.getInstance().getHeadingRotation());
        }
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if(Timer.getFPGATimestamp() - startTime < TURN_TIME) {
            super.initialize();
            Pose2d pose = trajectory.getStates().get(0).poseMeters;
            ChassisSpeeds chassis = ChassisSpeeds.fromFieldRelativeSpeeds(pose.getX()*0.00001, pose.getY()*0.00001, 0, Rotation2d.fromDegrees(Drivetrain.getInstance().getHeading()));
            Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(chassis), false);
        }
        else {  
        super.execute();
        SmartDashboard.putNumber("mp x error", xController.getPositionError());
        SmartDashboard.putNumber("mp y error", yController.getPositionError());
        SmartDashboard.putNumber("mp theta error", thetaController.getPositionError());
        SmartDashboard.putNumber("mp theta target position", thetaController.getSetpoint().position);
        SmartDashboard.putNumber("mp theta target velocity", thetaController.getSetpoint().velocity);
        }
    }

    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
        SwerveManual.pigeonAngle = Drivetrain.getInstance().getHeading();
        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0))), true);
    }
}