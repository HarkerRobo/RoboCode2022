package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Limelight;
import harkerrobolib.commands.IndefiniteCommand;

public class AlignWithLimelight extends IndefiniteCommand {
    private static final double LIMELIGHT_THRESHOLD = 2;
    public static final double LIMELIGHT_KP = SwerveManual.LIMELIGHT_KP;
    public static final double LIMELIGHT_KD = SwerveManual.LIMELIGHT_KD;
    private ProfiledPIDController txController;

    public AlignWithLimelight() {
        addRequirements(Drivetrain.getInstance());
        txController = new ProfiledPIDController(LIMELIGHT_KP, 0, LIMELIGHT_KD,  new Constraints(4, 4));

    }

    public void execute() {
        Limelight.update();
        double angularVelocity = -txController.calculate(Limelight.getTx(), 0);
        ChassisSpeeds chassis = new ChassisSpeeds(0, 0, -angularVelocity);
        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(chassis), false);
    }

    public boolean isFinished(){
        return Math.abs(Limelight.getTx()) < LIMELIGHT_THRESHOLD;
    }

    public void end(boolean isFinished){
        System.out.println("end");
        ChassisSpeeds chassis = new ChassisSpeeds(0,0,0);
        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(chassis), false);
    }
}
