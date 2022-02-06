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
    private static PIDController xController = new PIDController(Drivetrain.MP_X_KP, Drivetrain.MP_X_KI, Drivetrain.MP_X_KD);
    private static PIDController yController = new PIDController(Drivetrain.MP_Y_KP, Drivetrain.MP_Y_KI, Drivetrain.MP_Y_KD);
    private static ProfiledPIDController thetaController = new ProfiledPIDController(Drivetrain.MP_THETA_KP, Drivetrain.MP_THETA_KI, Drivetrain.MP_X_KD, new Constraints(2 * Math.PI, 3 * Math.PI));
    private static final double TURN_TIME = 0.3;
    
    private Rotation2d initHeading;
    Trajectory trajectory;
    private double startTime;
    private boolean isFirst;

  
    public HSSwerveDriveController(Trajectory trajectory, Rotation2d initHeading, boolean isFirst) {
        super(trajectory, Drivetrain.getInstance().getOdometry()::getPoseMeters, 
                Drivetrain.getInstance().getKinematics(), xController, yController, thetaController,() -> initHeading, 
                Drivetrain.getInstance()::setAngleAndDriveVelocity, Drivetrain.getInstance());
        this.trajectory = trajectory;
        this.initHeading = initHeading;
        this.isFirst = isFirst;
    }

    public HSSwerveDriveController(Trajectory trajectory, Rotation2d initHeading) {
        this(trajectory, initHeading, false);
    }
    
    @Override
    public void initialize() {
        super.initialize();
        if(isFirst) {
            Drivetrain.getInstance().getPigeon().addFusedHeading(-63.9886 * (Drivetrain.getInstance().getPigeon().getFusedHeading()+initHeading.getDegrees()));
            Drivetrain.getInstance().getOdometry().resetPosition(new Pose2d(trajectory.getInitialPose().getTranslation(),Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getFusedHeading())),
                Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getFusedHeading()));
        }
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if(Timer.getFPGATimestamp() - startTime < TURN_TIME) {
            super.initialize();
            Pose2d pose = trajectory.getStates().get(0).poseMeters;
            ChassisSpeeds chassis = ChassisSpeeds.fromFieldRelativeSpeeds(pose.getX()*0.00001, pose.getY()*0.00001, 0, Rotation2d.fromDegrees(-Drivetrain.getInstance().getPigeon().getFusedHeading()));
            Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(chassis), false);
        }
        else {
        super.execute();
        SmartDashboard.putNumber("mp x error", xController.getPositionError());
        SmartDashboard.putNumber("mp y error", yController.getPositionError());
        SmartDashboard.putNumber("mp theta error", thetaController.getPositionError());
        }
    }

    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
        SwerveManual.pigeonAngle = Drivetrain.getInstance().getPigeon().getFusedHeading();
        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0))), true);
    }
}