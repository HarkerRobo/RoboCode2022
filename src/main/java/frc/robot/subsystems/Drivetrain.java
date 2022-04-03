package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Units;
import frc.robot.util.SwerveModule;

/**
 * Specifies a drivetrain with 4 swerve modules
 */
public class Drivetrain extends SubsystemBase {
    private static Drivetrain drivetrain;

    private SwerveModule[] modules;

    private static final boolean[] ROTATION_INVERT = (RobotMap.IS_COMP) ? new boolean[]{false, false, false, false} : new boolean[]{false, false, false, false};
    private static final boolean[] TRANSLATION_INVERT =(RobotMap.IS_COMP) ? new boolean[]{true, true, true, false} : new boolean[]{false, true, true, true};

    public static final double[] OFFSETS = (RobotMap.IS_COMP) ? new double[]{285.468750-90, 178.330078, 109.951172, 32.255859} : new double[]{263.935547,270.175781+90,285.205078,331.699219};

    public static final boolean IS_PIGEON_UP = (RobotMap.IS_COMP) ? true : true;

    public static final double DT_WIDTH = 0.5461; // 0.93345 bumper to bumper
    public static final double DT_LENGTH = 0.5969; // 0.88265

    public static final double VOLTAGE_COMP = 10;

    public static final double TRANSLATION_GEAR_RATIO = 6.75;
    public static final double ROTATION_GEAR_RATIO = 12.8; // 12.8 rotations of motor = 1 rotation of wheel
    public static final double WHEEL_DIAMETER = 3.872; //inches
    public static final double FEET_TO_METER = 0.3048;

    public static final double MIN_OUTPUT = 0.01;
    public static final double MAX_DRIVE_VEL = 3; // theoretical 4.1148 m / s
    public static final double MAX_DRIVE_ACC = 30;

    public static final double MAX_ANGULAR_VEL = Math.PI*1.7;

    private boolean fieldCentric = true;
    private double pigeonPitchVel = 0.0;
    private double lastPigeonPitch = 0.0;
    private double prevPitchVel = 0.0;

    private WPI_Pigeon2 pigeon;

    private SwerveDriveKinematics kinematics;
    private SwerveDriveOdometry odometry;

    private Drivetrain() {
        modules = new SwerveModule[4];
        for(int i = 0; i < 4; i++){
            modules[i] = new SwerveModule(RobotMap.ROTATION_IDS[i], RobotMap.ROTATION_CANCODER_IDS[i], RobotMap.TRANSLATION_IDS[i], 
                ROTATION_INVERT[i], TRANSLATION_INVERT[i]);
        }
        
        pigeon = new WPI_Pigeon2(RobotMap.PIGEON_ID, RobotMap.CANIVORE);
        pigeon.setYaw(0);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 255);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.RawStatus_4_Mag, 255);
        // pigeon.setFusedHeading(0);//-RobotMap.PIGEON_CONSTANT * pigeon.getYaw());
        
        kinematics = new SwerveDriveKinematics(new Translation2d(DT_LENGTH / 2, DT_WIDTH / 2), new Translation2d(DT_LENGTH / 2, -DT_WIDTH / 2), 
        new Translation2d(-DT_LENGTH / 2, DT_WIDTH / 2), new Translation2d(-DT_LENGTH / 2, -DT_WIDTH / 2));
        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(pigeon.getYaw()), new Pose2d(7.65, 1.86, Rotation2d.fromDegrees(-90)));
        lastPigeonPitch = pigeon.getPitch();
    }

    /**
     * Set angle to given angle and set velocity to given velocity
     */
    public void setAngleAndDriveVelocity(SwerveModuleState[] states, boolean isPercentOutput){
        for(int i = 0; i < 4; i++){
            modules[i].setSwerveManual(states[i], isPercentOutput);
        }
    }

    public void setAngle(SwerveModuleState[] states){
        for(int i = 0; i < 4; i++){
            states[i].speedMetersPerSecond = 0;
            modules[i].setSwerveManual(states[i], true);
        }
    }
//battery boi was here
    public void setAngleAndDriveVelocity(SwerveModuleState[] states) {
        setAngleAndDriveVelocity(states, false);
    }

    public void toggleFieldCentric() {
        fieldCentric = fieldCentric == false;
    }

    public boolean isFieldCentric() {
        return fieldCentric;
    }

    public WPI_Pigeon2 getPigeon(){
        return pigeon;
    }

    public double getHeading(){
        return (!IS_PIGEON_UP) ? -pigeon.getYaw() : pigeon.getYaw();
    }

    public double getPitchVel(){
        return pigeonPitchVel;
    }

    public void updatePitchVel(){
        double pigeonPitch = pigeon.getPitch();
        prevPitchVel = pigeonPitchVel;
        pigeonPitchVel = pigeonPitch - lastPigeonPitch;
        lastPigeonPitch = pigeonPitch;
    }

    public double getPrevPitchVel() {
        return prevPitchVel;
    }

    public Rotation2d getHeadingRotation() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public SwerveDriveOdometry getOdometry() {
        return odometry;
    }

    public SwerveDriveKinematics getKinematics(){
        return kinematics;
    }
    
    public SwerveModule getTopLeft() {
        return modules[0];
    }

    public SwerveModule getTopRight() {
        return modules[1];
    }

    public SwerveModule getBottomLeft() {
        return modules[2];
    }

    public SwerveModule getBottomRight() {
        return modules[3];
    }

    public static Drivetrain getInstance() {
        if (drivetrain == null) {
            drivetrain = new Drivetrain();
        }
        return drivetrain;
    }

    /**
     * we need to know where 0 is after turning off the robot --> set encoder position for rotation gear from
     * Can Coder (absolute encoder) to offset
     */
    public void readCANCoders() {
        for(int i = 0; i < 4; i++){
            modules[i].getRotationMotor().setSelectedSensorPosition(
                (modules[i].getCanCoder().getAbsolutePosition() - OFFSETS[i]) / Units.FALCON_ENCODER_TO_DEGREE * ROTATION_GEAR_RATIO);
        }
    }

    public void destroyCANCoders() {
        for(int i=0; i < 4; i++)
            modules[i].getCanCoder().DestroyObject();
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Drivetrain");
        builder.addDoubleProperty("Heading", this::getHeading, null);

        builder.addDoubleProperty("Top Left CAN Coder Pos", getTopLeft().getCanCoder()::getAbsolutePosition, null);
        builder.addDoubleProperty("Top Right CAN Coder Pos", getTopRight().getCanCoder()::getAbsolutePosition, null);
        builder.addDoubleProperty("Bottom Left CAN Coder Pos", getBottomLeft().getCanCoder()::getAbsolutePosition, null);
        builder.addDoubleProperty("Bottom Right CAN Coder Pos", getBottomRight().getCanCoder()::getAbsolutePosition, null);

        builder.addDoubleProperty("Top Left Angle", getTopLeft()::getRotationAngle, null);
        builder.addDoubleProperty("Top Right Angle", getTopRight()::getRotationAngle, null);
        builder.addDoubleProperty("Bottom Left Angle", getBottomLeft()::getRotationAngle, null);
        builder.addDoubleProperty("Bottom Right Angle", getBottomRight()::getRotationAngle, null);

        builder.addDoubleProperty("Top Left Target Angle", () -> getTopLeft().rotation.getClosedLoopTarget() * 
            Units.FALCON_ENCODER_TO_DEGREE / Drivetrain.ROTATION_GEAR_RATIO, null);
        builder.addDoubleProperty("Top Right Target Angle", () -> getTopRight().rotation.getClosedLoopTarget() * 
            Units.FALCON_ENCODER_TO_DEGREE / Drivetrain.ROTATION_GEAR_RATIO, null);
        builder.addDoubleProperty("Bottom Left Target Angle", () -> getBottomLeft().rotation.getClosedLoopTarget() * 
            Units.FALCON_ENCODER_TO_DEGREE / Drivetrain.ROTATION_GEAR_RATIO, null);
        builder.addDoubleProperty("Bottom Right Target Angle", () -> getBottomRight().rotation.getClosedLoopTarget() * 
            Units.FALCON_ENCODER_TO_DEGREE / Drivetrain.ROTATION_GEAR_RATIO, null);

        builder.addDoubleProperty("Top Left Angle Error", () -> getTopLeft().rotation.getClosedLoopError() * 
            Units.FALCON_ENCODER_TO_DEGREE / Drivetrain.ROTATION_GEAR_RATIO, null);
        builder.addDoubleProperty("Top Right Angle Error", () -> getTopRight().rotation.getClosedLoopError() * 
            Units.FALCON_ENCODER_TO_DEGREE / Drivetrain.ROTATION_GEAR_RATIO, null);
        builder.addDoubleProperty("Bottom Left Angle Error", () -> getBottomLeft().rotation.getClosedLoopError() * 
            Units.FALCON_ENCODER_TO_DEGREE / Drivetrain.ROTATION_GEAR_RATIO, null);
        builder.addDoubleProperty("Bottom Right Angle Error", () -> getBottomRight().rotation.getClosedLoopError() * 
            Units.FALCON_ENCODER_TO_DEGREE / Drivetrain.ROTATION_GEAR_RATIO, null);

        builder.addDoubleProperty("Odometry X", getOdometry().getPoseMeters()::getX, null);
        builder.addDoubleProperty("Odometry Y", getOdometry().getPoseMeters()::getY, null);
        builder.addDoubleProperty("Odometry Theta Radians", getOdometry().getPoseMeters().getRotation()::getRadians, null);
        builder.addDoubleProperty("Odometry Theta Degrees", getOdometry().getPoseMeters().getRotation()::getDegrees, null);

        builder.addDoubleProperty("Top Left Kalman Speed", getTopLeft()::getTranslationVelocity, null);
        builder.addDoubleProperty("Top Right Kalman Speed", getTopRight()::getTranslationVelocity, null);
        builder.addDoubleProperty("Bottom Left Kalman Speed", getBottomLeft()::getTranslationVelocity, null);
        builder.addDoubleProperty("Bottom Right Kalman Speed", getBottomRight()::getTranslationVelocity, null);

        builder.addDoubleProperty("Top Left Target Speed",  () -> getTopLeft().getTranslationLoop().getLinearSystemLoop().getNextR(0), null);
        builder.addDoubleProperty("Top Right Target Speed",  () -> getTopRight().getTranslationLoop().getLinearSystemLoop().getNextR(0), null);
        builder.addDoubleProperty("Bottom Left Target Speed", () -> getBottomLeft().getTranslationLoop().getLinearSystemLoop().getNextR(0), null);
        builder.addDoubleProperty("Bottom Right Target Speed", () -> getBottomRight().getTranslationLoop().getLinearSystemLoop().getNextR(0), null);

        builder.addDoubleProperty("Top Left Percent Output", getTopLeft().getTranslationLoop()::getOutput, null);
        builder.addDoubleProperty("Top Right Percent Output", getTopRight().getTranslationLoop()::getOutput, null);
        builder.addDoubleProperty("Bottom Left Percent Output", getBottomLeft().getTranslationLoop()::getOutput, null);
        builder.addDoubleProperty("Bottom Right Percent Output", getBottomRight().getTranslationLoop()::getOutput, null);

        builder.addDoubleProperty("Top Left Error",  () -> getTopLeft().getTranslationLoop().getLinearSystemLoop().getError(0), null);
        builder.addDoubleProperty("Top Right Error",  () -> getTopRight().getTranslationLoop().getLinearSystemLoop().getError(0), null);
        builder.addDoubleProperty("Bottom Left Error", () -> getBottomLeft().getTranslationLoop().getLinearSystemLoop().getError(0), null);
        builder.addDoubleProperty("Bottom Right Error", () -> getBottomRight().getTranslationLoop().getLinearSystemLoop().getError(0), null);
    }
}