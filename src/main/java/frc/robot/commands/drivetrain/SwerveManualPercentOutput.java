package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;

import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.OI;

/**
 * Command for SwerveModules that uses joystick inputs to drive with percent output.
 * It also has pigeon PID, which prevents the robot heading from drifting.
 */

public class SwerveManualPercentOutput extends IndefiniteCommand {
    private static final double OUTPUT_MULTIPLIER= 0.3;
    private static final double ANGLE_KP = 0;

    private static double pigeonAngle=Drivetrain.getInstance().getPigeon().getFusedHeading();

    private PIDController anglePID;

    public SwerveManualPercentOutput() {
        addRequirements(Drivetrain.getInstance());
        anglePID = new PIDController(ANGLE_KP, 0, 0);
    }

    @Override
    public void execute() {
        double angularVelocity = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.DEADBAND);
        double translationy = -MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.DEADBAND);
        double translationx = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.DEADBAND);
        double chasisMagnitude = Math.sqrt(Math.pow(translationx,2) + Math.pow(translationy,2));
        

        if(chasisMagnitude < Drivetrain.MIN_OUTPUT){
            translationx = 0;
            translationy = 0;
            if (Math.abs(angularVelocity) > Drivetrain.MIN_OUTPUT) {
                pigeonAngle = Drivetrain.getInstance().getPigeon().getFusedHeading();
            }
            angularVelocity = -anglePID.calculate(Drivetrain.getInstance().getPigeon().getFusedHeading(), pigeonAngle);
        }
    
        // if(OI.getInstance().getDriverGamepad().getButtonBumperRightState()){
        //     angularVelocity = -pid.calculate(Limelight.getTx(), TX_SETPOINT);
        
        // if(OI.getInstance().getDriverGamepad().getButtonBState() || OI.getInstance().getOperatorGamepad().getButtonBState()){
        //     Shooter.getInstance().setAutoHoodAngle();
        // }    

        angularVelocity *= Drivetrain.MAX_ANGULAR_VEL;
        translationx *= Drivetrain.MAX_DRIVE_VEL * OUTPUT_MULTIPLIER;
        translationy *= Drivetrain.MAX_DRIVE_VEL * OUTPUT_MULTIPLIER;

        if (OI.getInstance().getDriverGamepad().getButtonBumperLeftState()) {
            translationx *= 0.4;
            translationy *= 0.4;
        }

        // pigeonAngle = Drivetrain.getInstance().getPigeon().getFusedHeading();
        System.out.println("a");
        ChassisSpeeds chassis = ChassisSpeeds.fromFieldRelativeSpeeds(translationx, translationy, -angularVelocity, new Rotation2d(Math.toRadians(Drivetrain.getInstance().getPigeon().getFusedHeading())));
        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(chassis), true);
    }
}
