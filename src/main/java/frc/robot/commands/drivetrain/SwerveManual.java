package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Limelight;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;
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
    public static double LIMELIGHT_KP = 0.05;
    public static double LIMELIGHT_KI = 0.01;
    public static double LIMELIGHT_IZONE = 3;
    public static double LIMELIGHT_KD = 0.00;

    public static double LIMELIGHT_KS = 0.1;
    private ProfiledPIDController txController;
    private SlewRateLimiter limiter = new SlewRateLimiter(3);

    // private double lastXVel = 0;
    // private double lastYVel = 0;
    
    public static double pigeonAngle;
    private static final double PIGEON_DELAY = 0.3;
    private Debouncer debouncer = new Debouncer(PIGEON_DELAY, DebounceType.kRising);

    public SwerveManual() {
        addRequirements(Drivetrain.getInstance());
        txController = new ProfiledPIDController(LIMELIGHT_KP, LIMELIGHT_KI, LIMELIGHT_KD, new Constraints(4, 4));
        txController.setGoal(0);
        txController.setIntegratorRange(-LIMELIGHT_IZONE, LIMELIGHT_IZONE);
    }

    @Override
    public void execute() {
        // txController.setP(SmartDashboard.getNumber("limelight align kP", LIMELIGHT_KP));
        // txController.setI(SmartDashboard.getNumber("limelight align kI", LIMELIGHT_KI));
        // txController.setD(SmartDashboard.getNumber("limelight align kD", LIMELIGHT_KD));
        // double izone = SmartDashboard.getNumber("limelight align izone", LIMELIGHT_IZONE);
        // txController.setIntegratorRange(-izone, izone);

        double angularVelocity = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.DEADBAND);
        angularVelocity *= Math.abs(angularVelocity);
        double translationy = -MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.DEADBAND);
        double translationx = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.DEADBAND);
       
        
        translationx *= Math.abs(translationx);
        translationy *= Math.abs(translationy);
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

        // double mag = Math.sqrt(translationx * translationx + translationy * translationy);
        // double limitedMag = limiter.calculate(mag);
        // if(Math.abs(mag) > 1e-3) {
        //     translationx = translationx / mag * limitedMag;
        //     translationy = translationy / mag * limitedMag;
        // }

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
            angularVelocity += LIMELIGHT_KS * Math.signum(angularVelocity);
            pigeonAngle = Drivetrain.getInstance().getHeading();
            SmartDashboard.putBoolean("holding pigeon angle", false);
            SmartDashboard.putNumber("ll error", txController.getPositionError());
            SmartDashboard.putNumber("limelight setpoint", txController.getSetpoint().position);
            SmartDashboard.putNumber("limelight goal", txController.getGoal().position);
        } else {
            txController.reset(0);
        }

        SmartDashboard.putNumber("angular vel", angularVelocity);
        ChassisSpeeds chassis;
        if(Drivetrain.getInstance().isFieldCentric())
            chassis = ChassisSpeeds.fromFieldRelativeSpeeds(translationx, translationy, -angularVelocity, Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getYaw()));
        else
            chassis = ChassisSpeeds.fromFieldRelativeSpeeds(translationx, translationy, -angularVelocity, Rotation2d.fromDegrees(0));

        ChassisSpeeds prevSpeed = Drivetrain.getInstance().getKinematics().toChassisSpeeds(
            Drivetrain.getInstance().getTopLeft().getState(),
            Drivetrain.getInstance().getTopRight().getState(), 
            Drivetrain.getInstance().getBottomLeft().getState(), 
                Drivetrain.getInstance().getBottomRight().getState());

            double ax = (chassis.vxMetersPerSecond - prevSpeed.vxMetersPerSecond) / 0.02;
            double ay = (chassis.vyMetersPerSecond - prevSpeed.vyMetersPerSecond) / 0.02;
            double mag = Math.sqrt(ax*ax + ay*ay);
        
            double limitedMag = mag;
            if(Math.abs(limitedMag) > 30) {
                limitedMag = 30 * Math.signum(limitedMag);
            }
        if(Math.abs(mag) > 3) {
            ax = ax / Math.abs(mag) * limitedMag; //mag * limitedMag;
            ay = ay / Math.abs(mag) * limitedMag;  //ay / mag * limitedMag;
        }

        chassis.vxMetersPerSecond = prevSpeed.vxMetersPerSecond + ax * 0.02;
        chassis.vyMetersPerSecond = prevSpeed.vyMetersPerSecond + ay * 0.02;
        SmartDashboard.putNumber("axsays", ax);
        SmartDashboard.putNumber("ays", ay);
        SmartDashboard.putNumber("speedxs", chassis.vxMetersPerSecond);
        SmartDashboard.putNumber("speedys", chassis.vyMetersPerSecond);

        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(chassis), false);
    }
}