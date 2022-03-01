package frc.robot.commands.drivetrain;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.commands.IndefiniteCommand;
import frc.robot.util.Limelight;

public class AlignWithLimelight extends WaitCommand{
    private static final double LIMELIGHT_THRESHOLD = 2;

    public AlignWithLimelight(double timeout) {
        super(timeout);
        addRequirements(Drivetrain.getInstance());
    }

    public void execute() {
        double angularVelocity = 0.03 * Limelight.getTx();
        ChassisSpeeds chassis = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, -angularVelocity, Drivetrain.getInstance().getHeadingRotation());
        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(chassis));
    }

    public boolean isFinished(){
        return Math.abs(Limelight.getTx()) < LIMELIGHT_THRESHOLD || super.isFinished();
    }

    public void end(boolean isFinished){
        super.end(isFinished);
        ChassisSpeeds chassis = new ChassisSpeeds(0, 0, 0);
        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(chassis));
    }
}
