package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.hal.EncoderJNI;
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
    private static final boolean HOOD_INVERTED = false;
    
    public static final double kS = 0.65899;    
    public static final double kV = 0.63766;
    public static final double kA = 0.068002;

    public static final double HOOD_KP = 0;
    public static final double HOOD_KI = 0;
    public static final double HOOD_KD = 0;

    public static final double HOOD_STALLING_CURRENT = 10;

    public static final double MAX_HOOD_RANGE = 100000;

    private static final double HOOD_CURRENT_CONTINUOUS = 10;
    private static final double HOOD_CURRENT_PEAK = 10;
    private static final double HOOD_CURRENT_PEAK_DUR = 0.05;
    
    private static final double MAX_CONTROL_EFFORT = 10; // volts 
    private static final double MODEL_STANDARD_DEVIATION = 3;
    private static final double ENCODER_STANDARD_DEVIATION = 0.1;

    public static final double SHOOTER_REV_TIME = 1.0;

    private SimpleVelocitySystem velocitySystem;
    
    private HSFalcon master;
    private HSFalcon follower;
    private HSFalcon hood;
    private Encoder encoder;
    
    private Shooter() {
        master = new HSFalcon(RobotMap.SHOOTER_MASTER);
        follower = new HSFalcon(RobotMap.SHOOTER_FOLLOWER);
        hood = new HSFalcon(RobotMap.HOOD);
        encoder = new Encoder(8, 9);
        initMotors();
        velocitySystem = new SimpleVelocitySystem(kS, kV, kA, MAX_CONTROL_EFFORT, MODEL_STANDARD_DEVIATION, ENCODER_STANDARD_DEVIATION, RobotMap.LOOP_TIME);
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

        hood.configOpenloopRamp(0.1);
        hood.config_kP(RobotMap.SLOT_INDEX, HOOD_KP);
        hood.config_kI(RobotMap.SLOT_INDEX, HOOD_KI);
        hood.config_kD(RobotMap.SLOT_INDEX, HOOD_KD);
        hood.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, HOOD_CURRENT_CONTINUOUS, HOOD_CURRENT_PEAK, HOOD_CURRENT_PEAK_DUR));

    }

    public void setPercentOutput(double speed) {
        master.set(ControlMode.PercentOutput, speed);
    }

    // raw encoder value
    public double getRawVelocity() {
        return encoder.getRate();
    }

    public double getWheelRPS() {
        return encoder.getRate() / 4096;
    }

    public SimpleVelocitySystem getVelocitySystem() {
        return velocitySystem;
    }

    public void setVelocity(double vel){
        velocitySystem.set(vel);
        velocitySystem.update(getWheelRPS());
        setPercentOutput(vel);//velocitySystem.getOutput());
    }

    public void setHood(double pos){
        hood.set(ControlMode.Position, pos/MAX_HOOD_RANGE);
    }

    public HSFalcon getMaster() {
        return master;
    }

    public HSFalcon getHood() {
        return hood;
    }

    public Encoder getEncoder() {
        return encoder;
    }

    public static Shooter getInstance() {
        if (shooter == null) {
            shooter = new Shooter();
        }
        return shooter;
    }
}
