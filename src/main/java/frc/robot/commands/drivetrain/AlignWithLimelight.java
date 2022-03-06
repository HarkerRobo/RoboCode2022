package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Limelight;
import harkerrobolib.commands.IndefiniteCommand;

public class AlignWithLimelight extends IndefiniteCommand{
    private static final double LIMELIGHT_THRESHOLD = 0.1;
    public static final double LIMELIGHT_KP = 0.1;
    public static final double LIMELIGHT_KD = 0.0007;
    private static PIDController txController;
    public AlignWithLimelight() {
        addRequirements(Drivetrain.getInstance());
        txController = new PIDController(LIMELIGHT_KP, 0, LIMELIGHT_KD);

    }

    public void execute() {
        Limelight.update();
        double angularVelocity = -txController.calculate(Limelight.getTx(), 0);
        ChassisSpeeds chassis = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, -angularVelocity, Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getYaw()));
        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(chassis), false);
    }

    public boolean isFinished(){
        return Math.abs(Limelight.getTx()) < LIMELIGHT_THRESHOLD;
    }

    public void end(boolean isFinished){
        ChassisSpeeds chassis = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getYaw()));
        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(chassis), false);
    }
}
