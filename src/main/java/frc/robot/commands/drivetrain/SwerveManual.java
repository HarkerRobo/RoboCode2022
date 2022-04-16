package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Limelight;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.RobotMap;

/**
 * Command for SwerveModules that uses joystick inputs to drive.
 * It also has pigeon PID, which prevents the robot heading from drifting.
 */

public class SwerveManual extends IndefiniteCommand {
    private static final double OUTPUT_MULTIPLIER = 1;
    private static final double PIGEON_KP = 0.03;
    public static final double LIMELIGHT_KP = 0.07;
    public static final double LIMELIGHT_KI = 0.01;
    public static final double LIMELIGHT_KD = 0.00000;
    public static final double LIMELIGHT_KS = 0.01;
    public static final double REGION_MIN_X = 0;
    public static final double REGION_MIN_Y = 0;
    public static final double REGION_MAX_X = 0;
    public static final double REGION_MAX_Y= 0;
    public static double limelightIZone = 3;

    private TimeInterpolatableBuffer<Rotation2d> pigBuffer;
    private ProfiledPIDController txController;

    // private double lastXVel = 0;
    // private double lastYVel = 0;
    
    public static double pigeonAngle;
    private static final double PIGEON_DELAY = 0.4;
    private Debouncer debouncer = new Debouncer(PIGEON_DELAY, DebounceType.kRising);
    private double translationXVel, translationYVel, angularVel, translationXAcc, translationYAcc;
    private boolean holdingPigeonAngle, aligningWithLimelight;
    private ChassisSpeeds output;

    public SwerveManual() {
        addRequirements(Drivetrain.getInstance());
        txController = new ProfiledPIDController(LIMELIGHT_KP, LIMELIGHT_KI, LIMELIGHT_KD, new Constraints(4, 4));
        txController.setGoal(0);
        txController.setIntegratorRange(-limelightIZone, limelightIZone);

        pigBuffer = TimeInterpolatableBuffer.createBuffer(1.5);
    }

    @Override
    public void execute() {
        SmartDashboard.putData(this);
        readJoySticks();
        squareOutputs();
        scaleOutputs();
        pigeonKP();
        limelightAlign();
        generateOutputChassisSpeeds();
        clampAcceleration();
        if(RobotMap.DEMO_MODE)
            limitRegion();

        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(output), false);
    }

    private void readJoySticks() {
        if(RobotMap.DEMO_MODE) {
            angularVel = (OI.getInstance().getDriverGamepad().getRawButton(7) ? 1.0 : 0.0) - 
                (OI.getInstance().getDriverGamepad().getRawButton(8)  ? 1.0 : 0.0);
            translationXVel = (OI.getInstance().getDriverGamepad().getRawButton(4) ? 1.0 : 0.0) - 
                (OI.getInstance().getDriverGamepad().getRawButton(3)  ? 1.0 : 0.0);
            translationYVel = (OI.getInstance().getDriverGamepad().getRawButton(2) ? 1.0 : 0.0) - 
                (OI.getInstance().getDriverGamepad().getRawButton(1)  ? 1.0 : 0.0);
        }
        else {
            angularVel = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.DEADBAND);
            translationYVel = -MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.DEADBAND);
            translationXVel = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.DEADBAND);
        }
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
            if (Math.abs(angularVel) < Drivetrain.MIN_OUTPUT) angularVel = 0.001;
        }
        angularVel *= Drivetrain.MAX_ANGULAR_VEL * OUTPUT_MULTIPLIER;
        translationXVel *= Drivetrain.MAX_DRIVE_VEL * OUTPUT_MULTIPLIER;
        translationYVel *= Drivetrain.MAX_DRIVE_VEL * OUTPUT_MULTIPLIER;
        if(RobotMap.DEMO_MODE) {
            translationXVel *= 0.4;
            translationYVel *= 0.4;
            angularVel *= 0.5;
        }
        else if(OI.getInstance().getDriverGamepad().getButtonBumperLeft().get()) {
            translationYVel *= 0.6;
            translationXVel *= 0.6;
        }
    }

    private void pigeonKP() {
        holdingPigeonAngle = debouncer.calculate(
            Math.abs(MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.DEADBAND)) < Drivetrain.MIN_OUTPUT);
        if(holdingPigeonAngle)
            angularVel = -PIGEON_KP * (pigeonAngle - Drivetrain.getInstance().getHeading());
        else 
            pigeonAngle = Drivetrain.getInstance().getHeading();
    }

    private void limelightAlign() {
        txController.setIntegratorRange(-limelightIZone, limelightIZone);
        aligningWithLimelight = (OI.getInstance().getDriverGamepad().getButtonBumperRightState() || 
            (RobotMap.DEMO_MODE && translationXVel < 0.01 && translationYVel < 0.01 && angularVel < 0.01)) && 
            Limelight.isTargetVisible();
        pigBuffer.addSample(Timer.getFPGATimestamp(), Drivetrain.getInstance().getHeadingRotation());
        if(aligningWithLimelight) {
            Limelight.update();
            double laggyTx = Limelight.getTx();
            double laggyRotation = pigBuffer.getSample(Timer.getFPGATimestamp() - Limelight.getTl()/1000.0 - 0.02*4).getDegrees();
            double estimatedCurrentTx = pigBuffer.getSample(Timer.getFPGATimestamp()).getDegrees() - laggyRotation + laggyTx;
            SmartDashboard.putNumber("lat comp tx", estimatedCurrentTx);
            SmartDashboard.putNumber("laggy  tx", laggyTx);
            angularVel = -txController.calculate(Limelight.getTx());
            angularVel += LIMELIGHT_KS * Math.signum(angularVel);
            // if(Math.abs(estimatedCurrentTx) < 0.3) {
            //     angularVel = 0;
            // }
            pigeonAngle = Drivetrain.getInstance().getHeading();
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
        if(mag > Drivetrain.MAX_DRIVE_ACC){
            translationXAcc = translationXAcc /mag * Drivetrain.MAX_DRIVE_ACC;
            translationYAcc = translationYAcc /mag * Drivetrain.MAX_DRIVE_ACC;
        }
        
        output.vxMetersPerSecond = prevSpeed.vxMetersPerSecond + translationXAcc * 0.02;
        output.vyMetersPerSecond = prevSpeed.vyMetersPerSecond + translationYAcc * 0.02;
    }

    private void limitRegion(){
        Translation2d currentPosition = Drivetrain.getInstance().getOdometry().getPoseMeters().getTranslation();
        if(currentPosition.getX() > REGION_MAX_X)
            output.vxMetersPerSecond = Math.min(output.vxMetersPerSecond, 0);
        else if(currentPosition.getX() < REGION_MIN_X)
            output.vxMetersPerSecond = Math.max(output.vxMetersPerSecond, 0);
        if(currentPosition.getY() < REGION_MAX_X)
            output.vyMetersPerSecond = Math.min(output.vyMetersPerSecond, 0);
        else if(currentPosition.getY() < REGION_MIN_X)
            output.vyMetersPerSecond = Math.max(output.vyMetersPerSecond, 0);
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
        builder.addBooleanProperty("Aligning With Limelight", () -> aligningWithLimelight, null);
        builder.addDoubleProperty("Limelight Align P", () -> txController.getP(), (double a) -> txController.setP(a));
        builder.addDoubleProperty("Limelight Align I", () -> txController.getI(), (double a) -> txController.setI(a));
        builder.addDoubleProperty("Limelight Align D", () -> txController.getD(), (double a) -> txController.setD(a));
        builder.addDoubleProperty("Limelight Align I Zone", () -> limelightIZone, (double a) -> limelightIZone = a);
        builder.addDoubleProperty("Limelight Error", () -> txController.getPositionError(), null);
        builder.addDoubleProperty("Limelight Goal", () -> txController.getGoal().position, null);
        builder.addDoubleProperty("Limelight Setpoint", () -> txController.getSetpoint().position, null);
    }
}