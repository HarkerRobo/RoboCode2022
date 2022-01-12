package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;
import frc.robot.util.Limelight;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.RobotMap;

public class SwerveTranslationAlign extends IndefiniteCommand {
    private static final double OUTPUT_MULTIPLIER= 1;
    private static final double kP=0.1;
    private static final double kI=0.0;//00002;
    private static final double kD=0.0;//02;
    private static final double TX_SETPOINT=0;
    private static final double I_ZONE = 0;
    private static double pigeonAngle = Drivetrain.getInstance().getPigeon().getFusedHeading();

    private PIDController pid;
    public SwerveTranslationAlign() {
        addRequirements(Drivetrain.getInstance());
        pid = new PIDController(kP, kI, kD);
        pid.setIntegratorRange(-I_ZONE, I_ZONE);
    }

    @Override
    public void execute() {
        double translationalVelocity = 0.0;
        translationalVelocity = -pid.calculate(Limelight.getTx(), TX_SETPOINT);

        translationalVelocity *= Drivetrain.MAX_DRIVE_VEL;

        SmartDashboard.putNumber("translational Velocity", translationalVelocity);

        pigeonAngle = Drivetrain.getInstance().getPigeon().getFusedHeading();

        ChassisSpeeds chassis = ChassisSpeeds.fromFieldRelativeSpeeds(translationalVelocity, 0, 0, new Rotation2d(Math.toRadians(Drivetrain.getInstance().getPigeon().getFusedHeading())));

        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(chassis));
    }
}
