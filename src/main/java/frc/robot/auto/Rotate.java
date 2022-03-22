package frc.robot.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;

public class Rotate extends WaitCommand{
    private double speed;

    public Rotate(double seconds, double speed) {
        super(seconds);
        addRequirements(Drivetrain.getInstance());
        this.speed = speed;
    }

    public void execute() {
        Drivetrain.getInstance().setAngleAndDriveVelocity(
            Drivetrain.getInstance().getKinematics().toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, speed, Drivetrain.getInstance().getHeadingRotation())));
    }

    public void end(boolean interrupted) {
        Drivetrain.getInstance().setAngleAndDriveVelocity(
            Drivetrain.getInstance().getKinematics().toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Drivetrain.getInstance().getHeadingRotation())));
    }
}
