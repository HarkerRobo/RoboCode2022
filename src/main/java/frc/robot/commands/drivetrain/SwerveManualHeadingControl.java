package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Limelight;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

/**
 * Controls the Swerve Modules using PercentOutput or Velociy for the drive motors and 
 * Position PID for the angle motors.
 * The left joystick controls translation (velocity direction and magnitude)
 * The right joystick's X axis controls rotation (angular velocity magnitude)
 * 
 * 'back' is defined as closest to the battery
 * 'left' is defined as left when standing at the back and looking forward
 * 
 * @author Shahzeb Lakhani
 * @author Jatin Kohli
 * @author Angela Jia
 * @author Chirag Kaushik
 * @author Anirudh Kotamraju
 * @author Arjun Dixit
 * @author Rohan Bhowmik
 * @since 2/16/20
 */
public class SwerveManualHeadingControl extends IndefiniteCommand {

    static {
        if (RobotMap.IS_COMP) {
            HIGH_VELOCITY_HEADING_MULTIPLIER = 0.16;
            LOW_VELOCITY_HEADING_MULTIPLIER = 0.09;
        } else {
            HIGH_VELOCITY_HEADING_MULTIPLIER = 0.17;
            LOW_VELOCITY_HEADING_MULTIPLIER = 0.17;
        }
    }
    
    private static final double OUTPUT_MULTIPLIER = 1;
    private static final boolean IS_PERCENT_OUTPUT = false;
    private static double HIGH_VELOCITY_HEADING_MULTIPLIER;
    private static double LOW_VELOCITY_HEADING_MULTIPLIER;
    private static final double ACCELERATION_HEADING_MULTIPLIER = 0;
    private static final double TURN_VEL_THRESHOLD = 160;
    
    private double translateX, translateY, headingX, headingY, headingAngle, turnMagnitude;
    
    private static double prevPigeonHeading;
    private static double prevTime;
    private static double prevVel;
    private static boolean pigeonFlag; //True if the Driver Right X input is non-zero
    private static double pigeonAngle;
    
    private static double lastPigeonUpdateTime; // seconds
    private static double turnVel;
    private static double turnAccel;
    
    public static PIDController headingController;
    public static boolean joystickFlag;
    public static boolean headingFlag;
    public static boolean flag;
    public static boolean prevHeadingFlag;

    private static boolean xPressed;
    private static boolean xFlag;
    private static boolean aPressed;
    private static boolean aFlag;
    
    public static boolean isNotOptimized;

    private static final double kP=0.02;
    private static final double kI=0.0;//00002;
    private static final double kD=0.0;//02;
    private static final double TX_SETPOINT=0;
    private static final double I_ZONE = 0;

    private PIDController pid;


    public SwerveManualHeadingControl() {
        addRequirements(Drivetrain.getInstance());
        headingController = new PIDController(Drivetrain.MANUAL_HEADING_KP, Drivetrain.MANUAL_HEADING_KI, Drivetrain.MANUAL_HEADING_KD);
        pigeonFlag = false;

        pid = new PIDController(kP, kI, kD);
        pid.setIntegratorRange(-I_ZONE, I_ZONE);

        // pigeonAngle = 90;
        // prevPigeonHeading = 90;
        prevTime = Timer.getFPGATimestamp();
        lastPigeonUpdateTime = Timer.getFPGATimestamp();
        prevPigeonHeading = Drivetrain.getInstance().getPigeon().getFusedHeading();
        prevVel = 0;
        xPressed = false;
        xFlag = false;
        aPressed = false;
        aFlag = false;
    }

    @Override
    public void initialize() {
        Drivetrain.getInstance().applyToAllDrive(
            (driveMotor) -> driveMotor.selectProfileSlot(Drivetrain.DRIVE_VELOCITY_SLOT, RobotMap.SLOT_INDEX)
        );

        Drivetrain.getInstance().applyToAllAngle(
            (angleMotor) -> angleMotor.selectProfileSlot(Drivetrain.ANGLE_POSITION_SLOT, RobotMap.SLOT_INDEX)
        );
        pigeonFlag = true;
        prevPigeonHeading = Drivetrain.getInstance().getPigeon().getFusedHeading();
        pigeonAngle = prevPigeonHeading;

        joystickFlag = false;
        headingFlag = false;
        isNotOptimized = false;
        flag = false;
        prevHeadingFlag = false;
    }

    @Override
    public void execute() {
        translateX = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.DEADBAND);
        translateY = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.DEADBAND);
        headingX = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), 0.25);
        headingY = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightY(), 0.25);

        if (headingY != 0 || headingX != 0) {
            headingAngle = Math.toDegrees(Math.atan2(headingY, headingX));
            headingAngle -= 90;
            SmartDashboard.putNumber("heading angle", headingAngle);

            while (Drivetrain.getInstance().getPigeon().getFusedHeading() - headingAngle > 180) {
                headingAngle += 360;
            }

            while (Drivetrain.getInstance().getPigeon().getFusedHeading() - headingAngle <- 180) {
                headingAngle -= 360;
            }

            headingFlag = true;
        } else {
            headingFlag = false;
        }

        if (headingFlag == false && prevHeadingFlag == true) {
            flag = true;
        } 
        else if (headingFlag == true) {
            flag = false;
        }
        
        turnMagnitude = -1 * headingController.calculate(Drivetrain.getInstance().getPigeon().getFusedHeading(), headingAngle);
        SmartDashboard.putNumber("turn mag", turnMagnitude);

        if (Math.abs(translateX) > 0 || Math.abs(translateY) > 0 || Math.abs(headingX) > 0 || Math.abs(headingY) > 0) {
            joystickFlag = true;
        }
        
        //scale input from joysticks
        translateX *= OUTPUT_MULTIPLIER * Drivetrain.MAX_DRIVE_VEL * ((OI.getInstance().getDriverGamepad().getButtonBumperLeft().get() == true) ? 0.4 : 1);
        translateY *= OUTPUT_MULTIPLIER * Drivetrain.MAX_DRIVE_VEL * ((OI.getInstance().getDriverGamepad().getButtonBumperLeft().get() == true) ? 0.4 : 1);
        turnMagnitude *= -1 * OUTPUT_MULTIPLIER * Drivetrain.MAX_ANGULAR_VEL;
        
        if(OI.getInstance().getDriverGamepad().getButtonBumperRightState()){
            turnMagnitude = pid.calculate(Limelight.getTx(), TX_SETPOINT);
            turnMagnitude *= Drivetrain.MAX_ANGULAR_VEL;
        }

        // double currentPigeonHeading = Drivetrain.getInstance().getPigeon().getFusedHeading();
        xFlag = xPressed && OI.getInstance().getDriverGamepad().getButtonX().get() == false;
        xPressed = OI.getInstance().getDriverGamepad().getButtonX().get();
        
        if (xFlag) {
            headingAngle = Drivetrain.getInstance().getPigeon().getFusedHeading() - 130;
        }
        aFlag = aPressed  == true && OI.getInstance().getDriverGamepad().getButtonA().get() == false;
        aPressed = OI.getInstance().getDriverGamepad().getButtonA().get();
        if (aFlag) {
            headingAngle = Drivetrain.getInstance().getPigeon().getFusedHeading() + 130;
        }
        if (xPressed) {
            turnMagnitude = -0.7 * OUTPUT_MULTIPLIER * Drivetrain.MAX_ANGULAR_VEL;
        }
        else if (aPressed) {
            turnMagnitude = 0.7 * OUTPUT_MULTIPLIER * Drivetrain.MAX_ANGULAR_VEL;
        }
        
        // if(pigeonFlag && turnMagnitude == 0) { //If there was joystick input but now there is not
            //     double velocityHeadingMultiplier = Math.abs(turnVel) > TURN_VEL_THRESHOLD ? HIGH_VELOCITY_HEADING_MULTIPLIER : LOW_VELOCITY_HEADING_MULTIPLIER;
            
            //     // account for momentum when turning
            //     pigeonAngle = currentPigeonHeading + turnVel * velocityHeadingMultiplier + turnAccel * ACCELERATION_HEADING_MULTIPLIER;
            // }
            
            // pigeonFlag = Math.abs(turnMagnitude) > 0; //Update pigeon flag
            
            // if(!pigeonFlag) { //If there is no joystick input currently
                //     // turnMagnitude = !RobotMap.IS_PRACTICE ? Drivetrain.PIGEON_kP * (pigeonAngle - currentPigeonHeading) : turnMagnitude;
                //     turnMagnitude = Drivetrain.PIGEON_kP * (pigeonAngle - currentPigeonHeading);
                // 
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            translateX * (OI.getInstance().getDriverGamepad().getButtonY().get() ? -1 : 1), translateY * (OI.getInstance().getDriverGamepad().getButtonY().get() ? -1 : 1), headingFlag || flag ? turnMagnitude : 0, Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getFusedHeading())
            );
        if (joystickFlag)    
        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(speeds));
                    // Now use this in our kinematics
                    
                
                    // if(Timer.getFPGATimestamp() - lastPigeonUpdateTime > 0.01) {
                        //     double currentTime = Timer.getFPGATimestamp();
                        //     double deltaTime = (double)(currentTime - prevTime);
                        
                        //     turnVel = (currentPigeonHeading - prevPigeonHeading) / deltaTime;
            
        //     turnAccel = (turnVel - prevVel) / deltaTime;

        //     prevPigeonHeading = currentPigeonHeading;
        //     prevVel = turnVel;
        //     prevTime = currentTime;
        //     lastPigeonUpdateTime = Timer.getFPGATimestamp();
        // }
        prevHeadingFlag = headingFlag;
    }

    @Override
    public void end(boolean interrupted) {
        Drivetrain.getInstance().stopAllDrive();
        headingFlag = false;
        joystickFlag = false;
    }
}