package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;

import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.OI;
import frc.robot.RobotMap;

public class SwerveManualPercentOutput extends IndefiniteCommand {
    private static final double OUTPUT_MULTIPLIER= 1;
    private static final double kP=0.03;

    private static double pigeonAngle=Drivetrain.getInstance().getPigeon().getFusedHeading();

    private PIDController pid;

    public SwerveManualPercentOutput() {
        addRequirements(Drivetrain.getInstance());
        pid = new PIDController(kP, 0, 0);
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
            if(Math.abs(angularVelocity) < Drivetrain.MIN_OUTPUT){
                angularVelocity = 0;
            }
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

        pigeonAngle = Drivetrain.getInstance().getPigeon().getFusedHeading();

        ChassisSpeeds chassis = ChassisSpeeds.fromFieldRelativeSpeeds(translationx, translationy, -angularVelocity, new Rotation2d(Math.toRadians(Drivetrain.getInstance().getPigeon().getFusedHeading())));
        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(chassis), true);
    }
}
