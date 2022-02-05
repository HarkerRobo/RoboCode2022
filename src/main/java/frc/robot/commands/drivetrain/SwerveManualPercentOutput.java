package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;

import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;

/**
 * Command for SwerveModules that uses joystick inputs to drive with percent output.
 * It also has pigeon PID, which prevents the robot heading from drifting.
 */

public class SwerveManualPercentOutput extends IndefiniteCommand {
    private static final double OUTPUT_MULTIPLIER= 0.3;
    private static final double PIGEON_KP = 0.1;

    private static double pigeonAngle;
    private static final double PIGEON_DELAY = 0.3;
    private Debouncer debouncer = new Debouncer(PIGEON_DELAY, DebounceType.kRising);

    public SwerveManualPercentOutput() {
        addRequirements(Drivetrain.getInstance());
    }

    @Override
    public void execute() {
        double angularVelocity = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.DEADBAND);
        double translationy = -MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.DEADBAND);
        double translationx = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.DEADBAND);
        double chasisMagnitude = Math.sqrt(Math.pow(translationx,2) + Math.pow(translationy,2));
        
        if(Math.abs(chasisMagnitude) < Drivetrain.MIN_OUTPUT){
            translationx = 0;
            translationy = 0;
            if (Math.abs(angularVelocity) < Drivetrain.MIN_OUTPUT) {
                angularVelocity = 0;//Drivetrain.getInstance().getPigeon().getFusedHeading();
            }
        }
    
        SmartDashboard.putNumber("trans X", translationx);

        angularVelocity *= Drivetrain.MAX_ANGULAR_VEL * OUTPUT_MULTIPLIER;
        translationx *= Drivetrain.MAX_DRIVE_VEL * OUTPUT_MULTIPLIER;
        translationy *= Drivetrain.MAX_DRIVE_VEL * OUTPUT_MULTIPLIER;

        // if (OI.getInstance().getDriverGamepad().getButtonBumperLeftState()) {
        //     translationx *= 0.4;
        //     translationy *= 0.4;
        // }

        if(debouncer.calculate(
            Math.abs(MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.DEADBAND)) < Drivetrain.MIN_OUTPUT)) {
            angularVelocity = -PIGEON_KP * (pigeonAngle - Drivetrain.getInstance().getPigeon().getFusedHeading());
            SmartDashboard.putBoolean("holding pigeon angle", true);
        }
        else {
            pigeonAngle = Drivetrain.getInstance().getPigeon().getFusedHeading();
            SmartDashboard.putBoolean("holding pigeon angle", false);
        }

        // ChassisSpeeds chassis = ChassisSpeeds.fromFieldRelativeSpeeds(translationx, translationy, -angularVelocity, new Rotation2d(Math.toRadians(Drivetrain.getInstance().getPigeon().getFusedHeading())));
        
        ChassisSpeeds chassis;
        if(Drivetrain.getInstance().isFieldCentric())
            chassis = ChassisSpeeds.fromFieldRelativeSpeeds(translationx, translationy, -angularVelocity, Rotation2d.fromDegrees(-Drivetrain.getInstance().getPigeon().getFusedHeading()));
        else
            chassis = ChassisSpeeds.fromFieldRelativeSpeeds(translationx, translationy, -angularVelocity, Rotation2d.fromDegrees(0));
        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(chassis), true);
    }
}
