package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Units;
import frc.robot.util.SimpleVelocitySystem;
import harkerrobolib.wrappers.HSFalcon;

/**
 * Specifies a two-motor shooter with a linear quadratic regulator and a kalman filter
 */
public class Shooter extends SubsystemBase {
    private static Shooter shooter;

    private static final boolean MASTER_INVERTED = true;
    private static final boolean FOLLOWER_INVERTED = false;
    private static final boolean HOOD_INVERTED = true;
    
    public static final double kS = 0;//0.67951;    
    public static final double kV = 0.15808;
    public static final double kA = 0.015239;

    public static final double HOOD_KP = 0.03; //change
    public static final double HOOD_KI = 0.0001; //change
    public static final double HOOD_KD = 0; //change
    public static final double HOOD_IZONE = 3000; //change
    public static final double HOOD_PID_TOLERANCE = 10; //change
    public static final double HOOD_GEAR_RATIO = 1; //change
    private static ProfiledPIDController hoodController;

    public static final double HOOD_STALLING_CURRENT = 10;

    private static double hoodMaxPos = 8192;
    private static double hoodEncoderOffset = 0;

    private static final double HOOD_CURRENT_CONTINUOUS = 10;
    private static final double HOOD_CURRENT_PEAK = 10;
    private static final double HOOD_CURRENT_PEAK_DUR = 0.05;
    
    private static final double MAX_ERROR = 0.1; // volts 
    private static final double MAX_CONTROL_EFFORT = 10; // volts 
    private static final double MODEL_STANDARD_DEVIATION = 0.00625;
    private static final double ENCODER_STANDARD_DEVIATION = 0.00025;

    public static final double SHOOTER_REV_TIME = 1.0;

    private SimpleVelocitySystem velocitySystem;
    
    private HSFalcon master;
    private HSFalcon follower;
    private HSFalcon hood;
    private Encoder shooterEncoder;
    private Counter hoodEncoder;
    
    private Shooter() {
        master = new HSFalcon(RobotMap.SHOOTER_MASTER);
        follower = new HSFalcon(RobotMap.SHOOTER_FOLLOWER);
        hood = new HSFalcon(RobotMap.HOOD);
        shooterEncoder = new Encoder(RobotMap.SHOOTER_ENCODER_A, RobotMap.SHOOTER_ENCODER_B);
        hoodEncoder = new Counter(RobotMap.HOOD_ENCODER);
        hoodEncoder.setSemiPeriodMode(true);
        hoodController = new ProfiledPIDController(HOOD_KP, HOOD_KI, HOOD_KD, new Constraints(3,10));
        hoodController.setIntegratorRange(-HOOD_IZONE, HOOD_IZONE);
        initMotors();
        velocitySystem = new SimpleVelocitySystem(kS, kV, kA, MAX_ERROR, MAX_CONTROL_EFFORT, MODEL_STANDARD_DEVIATION, ENCODER_STANDARD_DEVIATION, RobotMap.LOOP_TIME);
    }

    public void initMotors() {
        master.configFactoryDefault();
        follower.configFactoryDefault();
        hood.configFactoryDefault();

        follower.follow(master);

        master.setInverted(MASTER_INVERTED);
        follower.setInverted(FOLLOWER_INVERTED);
        hood.setInverted(HOOD_INVERTED);

        master.configVelocityMeasurementWindow(1);
        master.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);

        master.configVoltageCompSaturation(MAX_CONTROL_EFFORT);
        follower.configVoltageCompSaturation(MAX_CONTROL_EFFORT);
        hood.configVoltageCompSaturation(MAX_CONTROL_EFFORT);

        // hood.configOpenloopRamp(0.1);
        // hood.config_kP(RobotMap.SLOT_INDEX, HOOD_KP);
        // hood.config_kI(RobotMap.SLOT_INDEX, HOOD_KI);
        // hood.config_kD(RobotMap.SLOT_INDEX, HOOD_KD);
        // hood.config_IntegralZone(RobotMap.SLOT_INDEX, HOOD_IZONE);
        hood.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, HOOD_CURRENT_CONTINUOUS, HOOD_CURRENT_PEAK, HOOD_CURRENT_PEAK_DUR));
        hood.configPeakOutputForward(0.5);
        hood.configPeakOutputReverse(-0.5);

    }

    public void setPercentOutput(double speed) {
        master.set(ControlMode.PercentOutput, speed);
    }

    public double getHoodEncoderPos(){
        return 1e6 * hoodEncoder.getPeriod() / Units.MAG_CODER_ENCODER_TICKS;
    }

    public void setHoodOffset() {
        hoodEncoderOffset = getHoodEncoderPos();
    }

    public void setHoodMax() {
        hoodMaxPos = getHoodPos();
    }

    public double getHoodPos(){
        double falconPos = hood.getSelectedSensorPosition() / Units.FALCON_ENCODER_TICKS / HOOD_GEAR_RATIO;
        double magPos = getHoodEncoderPos() - hoodEncoderOffset;
        while(falconPos - magPos > 0.5)
            magPos++;
        return magPos;
    }

    // raw encoder value
    public double getRawVelocity() {
        return shooterEncoder.getRate();
    }

    public double getWheelRPS() {
        return shooterEncoder.getRate() / 1024;
    }

    public SimpleVelocitySystem getVelocitySystem() {
        return velocitySystem;
    }

    public ProfiledPIDController getHoodController() {
        return hoodController;
    }

    public void setVelocity(double vel){
        velocitySystem.set(vel);
        velocitySystem.update(getWheelRPS());
        // setPercentOutput(vel);
        setPercentOutput(velocitySystem.getOutput());
    }

    public void setHood(double pos){
        hood.set(ControlMode.PercentOutput, hoodController.calculate(getHoodPos(), 
            pos * hoodMaxPos));
    }

    public HSFalcon getMaster() {
        return master;
    }

    public HSFalcon getHood() {
        return hood;
    }

    public Encoder getShooterEncoder() {
        return shooterEncoder;
    }

    public static Shooter getInstance() {
        if (shooter == null) {
            shooter = new Shooter();
        }
        return shooter;
    }
}
