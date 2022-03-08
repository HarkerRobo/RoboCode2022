package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Limelight;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;

/**
 * Command for SwerveModules that uses joystick inputs to drive.
 * It also has pigeon PID, which prevents the robot heading from drifting.
 */

public class SwerveManual extends IndefiniteCommand {
    private static final double OUTPUT_MULTIPLIER = 1;
    private static final double PIGEON_KP = 0.03;
    public static final double LIMELIGHT_KP = 0.12;
    public static final double LIMELIGHT_KD = 0.01;
    private ProfiledPIDController txController;
    private SlewRateLimiter limiter = new SlewRateLimiter(3);
    
    public static double pigeonAngle;
    private static final double PIGEON_DELAY = 0.3;
    private Debouncer debouncer = new Debouncer(PIGEON_DELAY, DebounceType.kRising);

    public SwerveManual() {
        addRequirements(Drivetrain.getInstance());
        txController = new ProfiledPIDController(LIMELIGHT_KP, 0, LIMELIGHT_KD, new Constraints(4, 4));
        txController.setGoal(0);
    }

    @Override
    public void execute() {
        double angularVelocity = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.DEADBAND);
        angularVelocity *= Math.abs(angularVelocity);
        double translationy = -MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.DEADBAND);
        double translationx = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.DEADBAND);
        double chasisMagnitude = Math.sqrt(Math.pow(translationx,2) + Math.pow(translationy,2));
        
        
        if(Math.abs(chasisMagnitude) < Drivetrain.MIN_OUTPUT){
            translationx = 0;
            translationy = 0;
            if (Math.abs(angularVelocity) < Drivetrain.MIN_OUTPUT) {
                angularVelocity = 0.001;
            }
        }
    
        SmartDashboard.putNumber("trans X", translationx);

        angularVelocity *= Drivetrain.MAX_ANGULAR_VEL * OUTPUT_MULTIPLIER;
        translationx *= Drivetrain.MAX_DRIVE_VEL * OUTPUT_MULTIPLIER;
        translationy *= Drivetrain.MAX_DRIVE_VEL * OUTPUT_MULTIPLIER;

        double mag = Math.sqrt(translationx * translationx + translationy * translationy);
        double limitedMag = limiter.calculate(mag);
        if(Math.abs(mag) > 1e-3) {
            translationx = translationx / mag * limitedMag;
            translationy = translationy / mag * limitedMag;
        }

        if(OI.getInstance().getDriverGamepad().getButtonBumperLeft().get()) {
            translationy *= 0.6;
            translationx *= 0.6;
        }

        if(debouncer.calculate(
            Math.abs(MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.DEADBAND)) < Drivetrain.MIN_OUTPUT)) {
            angularVelocity = -PIGEON_KP * (pigeonAngle - Drivetrain.getInstance().getHeading());
            SmartDashboard.putBoolean("holding pigeon angle", true);
        }
        else {  
            pigeonAngle = Drivetrain.getInstance().getHeading();
            SmartDashboard.putBoolean("holding pigeon angle", false);
        }
        if(OI.getInstance().getDriverGamepad().getButtonBumperRightState() && Limelight.isTargetVisible()) {
            Limelight.update();
            
            angularVelocity = -txController.calculate(Limelight.getTx());
            pigeonAngle = Drivetrain.getInstance().getHeading();
            SmartDashboard.putBoolean("holding pigeon angle", false);
            SmartDashboard.putNumber("limelight setpoint", txController.getSetpoint().position);
            SmartDashboard.putNumber("limelight goal", txController.getGoal().position);
        } else {
            // txController.reset(0);
        }

        SmartDashboard.putNumber("angular vel", angularVelocity);
        ChassisSpeeds chassis;
        if(Drivetrain.getInstance().isFieldCentric())
            chassis = ChassisSpeeds.fromFieldRelativeSpeeds(translationx, translationy, -angularVelocity, Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getYaw()));
        else
            chassis = ChassisSpeeds.fromFieldRelativeSpeeds(translationx, translationy, -angularVelocity, Rotation2d.fromDegrees(0));
        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(chassis), false);
    }
}
