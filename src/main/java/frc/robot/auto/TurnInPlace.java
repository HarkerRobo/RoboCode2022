package frc.robot.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.drivetrain.HSSwerveDriveController;
import frc.robot.subsystems.Drivetrain;

public class TurnInPlace extends CommandBase{
    private double turn;
    private ProfiledPIDController thetaController;
    private static final double TOLERANCE = 0.05;

    public TurnInPlace(double setpoint) {
        addRequirements(Drivetrain.getInstance());
        turn = setpoint*Math.PI/180;
        thetaController = new ProfiledPIDController(HSSwerveDriveController.THETA_KP, HSSwerveDriveController.THETA_KI, HSSwerveDriveController.THETA_KD, new Constraints(HSSwerveDriveController.MAX_ANGLE_VELOCITY, HSSwerveDriveController.MAX_ANGLE_ACCELERATION));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

    }
    public void initialize() {
        thetaController.setP(4.7);
        thetaController.setI(0.01);
        thetaController.setD(0);
        thetaController.setGoal(turn);
    }


    public void execute() {
        double angularVelocity = thetaController.calculate(Drivetrain.getInstance().getHeadingRotation().getRadians());
        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(0,0, -angularVelocity, Drivetrain.getInstance().getHeadingRotation())));
    }

    public boolean isFinished() {
        return Math.abs(thetaController.getPositionError()) < TOLERANCE;
    }

    public void end(boolean interrupted) {
        ChassisSpeeds chassis = new ChassisSpeeds(0, 0, 0);
        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(chassis));
    }


    

}
