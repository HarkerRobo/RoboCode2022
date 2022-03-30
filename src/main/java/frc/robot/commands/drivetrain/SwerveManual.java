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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;

/**
 * Command for SwerveModules that uses joystick inputs to drive.
 * It also has pigeon PID, which prevents the robot heading from drifting.
 */

public class SwerveManual extends IndefiniteCommand {
    private static final double OUTPUT_MULTIPLIER = 1;
    private static final double PIGEON_KP = 0.03;
    public static double LIMELIGHT_KP = 0.08;
    public static double LIMELIGHT_KI = 0.01;
    public static double LIMELIGHT_IZONE = 3;
    public static double LIMELIGHT_KD = 0.01;

    private ProfiledPIDController txController;

    // private double lastXVel = 0;
    // private double lastYVel = 0;
    
    public static double pigeonAngle;
    private static final double PIGEON_DELAY = 0.3;
    private Debouncer debouncer = new Debouncer(PIGEON_DELAY, DebounceType.kRising);
    private double translationXVel;
    private double translationYVel;
    private double angularVel;
    private double translationXAcc;
    private double translationYAcc;
    private boolean holdingPigeonAngle;
    private ChassisSpeeds output;

    public SwerveManual() {
        addRequirements(Drivetrain.getInstance());
        txController = new ProfiledPIDController(LIMELIGHT_KP, LIMELIGHT_KI, LIMELIGHT_KD, new Constraints(4, 4));
        txController.setGoal(0);
        txController.setIntegratorRange(-LIMELIGHT_IZONE, LIMELIGHT_IZONE);
    }

    @Override
    public void execute() {
        readJoySticks();
        squareOutputs();
        scaleOutputs();
        pigeonKP();
        limelightAlign();
        generateOutputChassisSpeeds();
        clampAcceleration();

        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(output), false);
    }

    private void readJoySticks() {
        angularVel = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.DEADBAND);
        translationYVel = -MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.DEADBAND);
        translationXVel = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.DEADBAND);
    }

    private void squareOutputs() {
        translationXVel *= Math.abs(translationXVel);
        translationYVel *= Math.abs(translationYVel);
        angularVel *= Math.abs(angularVel);
    }

    private void scaleOutputs(){
        double chasisMagnitude = Math.sqrt(Math.pow(translationXVel,2) + Math.pow(translationYVel,2));
        if(Math.abs(chasisMagnitude) < Drivetrain.MIN_OUTPUT){
            translationXVel = 0;
            translationYVel = 0;
            if (Math.abs(angularVel) < Drivetrain.MIN_OUTPUT) {
                angularVel = 0.001;
            }
        }
        angularVel *= Drivetrain.MAX_ANGULAR_VEL * OUTPUT_MULTIPLIER;
        translationXVel *= Drivetrain.MAX_DRIVE_VEL * OUTPUT_MULTIPLIER;
        translationYVel *= Drivetrain.MAX_DRIVE_VEL * OUTPUT_MULTIPLIER;
        if(OI.getInstance().getDriverGamepad().getButtonBumperLeft().get()) {
            translationYVel *= 0.6;
            translationXVel *= 0.6;
        }
    }

    private void pigeonKP() {
        if(debouncer.calculate(
            Math.abs(MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.DEADBAND)) < Drivetrain.MIN_OUTPUT)) {
            angularVel = -PIGEON_KP * (pigeonAngle - Drivetrain.getInstance().getHeading());
            holdingPigeonAngle = true;
        }
        else {  
            pigeonAngle = Drivetrain.getInstance().getHeading();
            holdingPigeonAngle = false;
        }
    }

    private void limelightAlign() {
        if(OI.getInstance().getDriverGamepad().getButtonBumperRightState() && Limelight.isTargetVisible()) {
            Limelight.update();
            
            angularVel = -txController.calculate(Limelight.getTx());
            pigeonAngle = Drivetrain.getInstance().getHeading();
             SmartDashboard.putNumber("ll error", txController.getPositionError());
            SmartDashboard.putNumber("limelight setpoint", txController.getSetpoint().position);
            SmartDashboard.putNumber("limelight goal", txController.getGoal().position);
        } 
        else {
            txController.reset(0);
        }
    }

    private void generateOutputChassisSpeeds() {
        if(Drivetrain.getInstance().isFieldCentric())
            output = ChassisSpeeds.fromFieldRelativeSpeeds(translationXVel, translationYVel, -angularVel, Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getYaw()));
        else
            output = ChassisSpeeds.fromFieldRelativeSpeeds(translationXVel, translationYVel, -angularVel, Rotation2d.fromDegrees(0));
    }

    private void clampAcceleration() {
        ChassisSpeeds prevSpeed = Drivetrain.getInstance().getKinematics().toChassisSpeeds(
            Drivetrain.getInstance().getTopLeft().getState(),
            Drivetrain.getInstance().getTopRight().getState(), 
            Drivetrain.getInstance().getBottomLeft().getState(), 
            Drivetrain.getInstance().getBottomRight().getState());
        double prevX = prevSpeed.vxMetersPerSecond;
        double prevY = prevSpeed.vyMetersPerSecond;
        translationXAcc = (output.vxMetersPerSecond - prevX) / 0.02;
        translationYAcc = (output.vyMetersPerSecond - prevY) / 0.02;
        double mag = Math.sqrt(translationXAcc*translationXAcc + translationYAcc*translationYAcc);
        double limitedMag = MathUtil.constrain(mag, -Drivetrain.MAX_DRIVE_ACC, Drivetrain.MAX_DRIVE_ACC);
        
        translationXAcc = translationXAcc /mag * limitedMag;
        translationYAcc = translationYAcc /mag * limitedMag;
        
        output.vxMetersPerSecond = prevX + translationXAcc * 0.02;
        output.vyMetersPerSecond = prevY + translationYAcc * 0.02;
    }

    public void initSendable(SendableBuilder builder)
    {
        builder.setSmartDashboardType("Swerve Manual");
        builder.addDoubleProperty("Translation X Vel", () -> translationXVel, null);
        builder.addDoubleProperty("Translation Y Vel", () -> translationYVel, null);
        builder.addDoubleProperty("Angular Vel", () -> angularVel, null);
        builder.addDoubleProperty("Translation X Acc", () -> translationXAcc, null);
        builder.addDoubleProperty("Translation Y Acc", () -> translationYAcc, null);
        builder.addBooleanProperty("Holding Pigeon Angle", () -> holdingPigeonAngle, null);
    }
}
