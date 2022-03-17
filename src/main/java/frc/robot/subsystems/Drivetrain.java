package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private static final boolean[] TRANSLATION_INVERT =(RobotMap.IS_COMP) ? new boolean[]{false, true, true, false} : new boolean[]{true, true, true, true};

    public static final double[] OFFSETS = (RobotMap.IS_COMP) ? new double[]{263.935547, 178.330078, 109.951172, 32.255859} : new double[]{270.175781,285.468750,285.205078,331.699219};

    public static final boolean IS_PIGEON_UP = (RobotMap.IS_COMP) ? true : true;

    public static final double DT_WIDTH = 0.5461; // 0.93345 bumper to bumper
    public static final double DT_LENGTH = 0.5969; // 0.88265

    public static final double VOLTAGE_COMP = 10;

    public static final double TRANSLATION_GEAR_RATIO = 6.75;
    public static final double ROTATION_GEAR_RATIO = 12.8; // 12.8 rotations of motor = 1 rotation of wheel
    public static final double WHEEL_DIAMETER = 3.872; //inches
    public static final double FEET_TO_METER = 0.3048;

    public static final double MIN_OUTPUT = 0.01;
    public static final double MAX_DRIVE_VEL = 3.5; // theoretical 4.1148 m / s
    public static final double MAX_ANGULAR_VEL = Math.PI*1.6;

    private boolean fieldCentric = true;

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
        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(pigeon.getYaw()), new Pose2d(0, 0, new Rotation2d()));
    }

    /**
     * Set angle to given angle and set velocity to given velocity
     */
    public void setAngleAndDriveVelocity(SwerveModuleState[] states, boolean isPercentOutput){
        for(int i = 0; i < 4; i++){
            SmartDashboard.putNumber("Desired angle module " + i, states[i].angle.getDegrees());
            SmartDashboard.putNumber("Desired translation speed " + i, states[i].speedMetersPerSecond);
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
}
