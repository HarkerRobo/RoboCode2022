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
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.Drivetrain;

public class HSSwerveDriveController extends SwerveControllerCommand {
    public static final double X_KP = 2.5;
    public static final double X_KI = 0;
    public static final double X_KD = 0;

    public static final double Y_KP = 2.5;
    public static final double Y_KI = 0;
    public static final double Y_KD = 0;

    public static final double THETA_KP = 3.7;
    public static final double THETA_KI = 0.01;
    public static final double THETA_KD = 0.0;

    public static final double MAX_ANGLE_VELOCITY = Math.PI/2+0.2;
    public static final double MAX_ANGLE_ACCELERATION = Math.PI/2;

    private static PIDController xController = new PIDController(X_KP, X_KI, X_KD);
    private static PIDController yController = new PIDController(Y_KP, Y_KI, Y_KD);
    private static ProfiledPIDController thetaController = 
        new ProfiledPIDController(THETA_KP, THETA_KI, THETA_KD, new Constraints(MAX_ANGLE_VELOCITY, MAX_ANGLE_ACCELERATION));
    private static final double TURN_TIME = 0.2;
    
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
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public HSSwerveDriveController(Trajectory trajectory, Rotation2d finalHeading) {
        this(trajectory, null, finalHeading, false);
    }
    
    @Override
    public void initialize() {
        super.initialize();
        if(isFirst && initHeading != null) {
            // Drivetrain.getInstance().getPigeon().setYaw(-initHeading.getDegrees()-90);
            Drivetrain.getInstance().getOdometry().resetPosition(new Pose2d(trajectory.getInitialPose().getTranslation(), 
                initHeading), 
                Drivetrain.getInstance().getHeadingRotation());
        }
        
        startTime = Timer.getFPGATimestamp();
    }


    @Override
    public void execute() {
        SmartDashboard.putData(this);
        if(Timer.getFPGATimestamp() - startTime < TURN_TIME) {
            super.initialize();
            Pose2d initialState = trajectory.sample(0).poseMeters;
            Pose2d nextState = trajectory.sample(0.1).poseMeters;
            double dx = nextState.getX() - initialState.getX();
            double dy = nextState.getY() - initialState.getY();
            ChassisSpeeds chassis = ChassisSpeeds.fromFieldRelativeSpeeds(dx*0.01, dy*0.01, 0, Drivetrain.getInstance().getHeadingRotation());
            Drivetrain.getInstance().setAngle(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(chassis));
            // thetaController.reset(Drivetrain.getInstance().getHeading());
            xController.reset();
            yController.reset();
        }
        else
            super.execute();
    }

    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
        SwerveManual.pigeonAngle = Drivetrain.getInstance().getHeading();
        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0))), true);
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Auton");
        builder.addDoubleProperty("X Error", xController::getPositionError, null);
        builder.addDoubleProperty("X Goal", xController::getSetpoint, null);
        builder.addDoubleProperty("Y Error", yController::getPositionError, null);
        builder.addDoubleProperty("Y Goal", yController::getSetpoint, null);
        builder.addDoubleProperty("Theta Error", thetaController::getPositionError, null);
        builder.addDoubleProperty("Theta Target Position", () -> thetaController.getSetpoint().position, null);
        builder.addDoubleProperty("Theta Target Velocity", () -> thetaController.getSetpoint().velocity, null);
    }
}