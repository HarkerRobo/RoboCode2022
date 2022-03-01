package frc.robot.commands.drivetrain;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.commands.IndefiniteCommand;
import frc.robot.util.Limelight;

public class AlignWithLimelight extends IndefiniteCommand{
    private static final double LIMELIGHT_THRESHOLD = 2;

    public AlignWithLimelight() {
        addRequirements(Drivetrain.getInstance());
    }

    public void execute() {
        double angularVelocity = 0.03 * Limelight.getTx();
        ChassisSpeeds chassis = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, -angularVelocity, Drivetrain.getInstance().getHeadingRotation());
        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(chassis));
    }

    public boolean isFinished(){
        return Math.abs(Limelight.getTx()) < LIMELIGHT_THRESHOLD;
    }

    public void end(boolean isFinished){
        ChassisSpeeds chassis = new ChassisSpeeds(0, 0, 0);
        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(chassis));
    }
}
