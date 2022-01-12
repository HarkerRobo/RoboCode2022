package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;
import frc.robot.util.Limelight;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.RobotMap;

public class SwerveManual extends IndefiniteCommand {
    private static final double OUTPUT_MULTIPLIER= 1;
    private static final double kP=0.03;
    private static final double kI=0.0;//00002;
    private static final double kD=0.0;//02;
    private static final double TX_SETPOINT=0;
    private static final double I_ZONE = 0;
    private static final double angleKP=1;

    private static double pigeonAngle=Drivetrain.getInstance().getPigeon().getFusedHeading();

    private PIDController pid;
    public SwerveManual() {
        addRequirements(Drivetrain.getInstance());
        pid = new PIDController(kP, kI, kD);
        pid.setIntegratorRange(-I_ZONE, I_ZONE);
    }

    @Override
    public void execute() {
        double angularVelocity = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.DEADBAND);
        double translationx = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.DEADBAND);
        double translationy = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.DEADBAND);
        double chasisMagnitude=Math.sqrt(translationx*translationx + translationy*translationy);
        

        if(chasisMagnitude<(Drivetrain.MIN_OUTPUT)){
            translationx=0;
            translationy=0;
            if(Math.abs(angularVelocity)<(Drivetrain.MIN_OUTPUT)){
            angularVelocity=0.000001;}
        }
    

        if(OI.getInstance().getDriverGamepad().getButtonBumperRightState()){
            angularVelocity = -pid.calculate(Limelight.getTx(), TX_SETPOINT);
        }
        // if(OI.getInstance().getDriverGamepad().getButtonBState() || OI.getInstance().getOperatorGamepad().getButtonBState()){
        //     Shooter.getInstance().setAutoHoodAngle();
        // }

        angularVelocity *= Drivetrain.MAX_ANGULAR_VEL;
        SmartDashboard.putNumber("limelight tx", Limelight.getTx());
        SmartDashboard.putNumber("limelight ang vel", angularVelocity);
        translationx *= Drivetrain.MAX_DRIVE_VEL * OUTPUT_MULTIPLIER;
        translationy *= Drivetrain.MAX_DRIVE_VEL * OUTPUT_MULTIPLIER;

        if (OI.getInstance().getDriverGamepad().getButtonBumperLeftState()) {
            translationx *= 0.4;
            translationy *= 0.4;
        }

        if(RobotMap.DEMO_MODE && !(OI.getInstance().getDriverGamepad().getButtonBumperLeftState() && OI.getInstance().getDriverGamepad().getButtonBumperRightState())){
            translationx *= 0.3;
            translationy *= 0.3;
        }
       // System.out.println(translationx + " " + translationy);
        // double rotation =
        // MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(),
        // OI.DEADBAND);

    
    // if(Math.abs(MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.DEADBAND))<0.05){
    //     angularVelocity=angleKP*(pigeonAngle - Drivetrain.getInstance().getPigeon().getFusedHeading());
    // }

        pigeonAngle=Drivetrain.getInstance().getPigeon().getFusedHeading();

        ChassisSpeeds chassis = ChassisSpeeds.fromFieldRelativeSpeeds(translationx, translationy, -angularVelocity, new Rotation2d(Math.toRadians(Drivetrain.getInstance().getPigeon().getFusedHeading())));
        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(chassis));
    }
}
