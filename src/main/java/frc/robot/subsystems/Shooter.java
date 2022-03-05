package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
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

    private static final boolean MASTER_INVERTED = (RobotMap.IS_COMP) ? true : true;
    private static final boolean FOLLOWER_INVERTED = (RobotMap.IS_COMP) ? false : false;
    
    public static final double kS = 0;//0.66342;    
    public static final double kV = 0.15911;
    public static final double kA = 0.019298;

    public static final double HOOD_STALLING_CURRENT = 10;
    
    private static final double MAX_ERROR = 0.1; 
    private static final double MODEL_STANDARD_DEVIATION = 0.1;
    private static final double ENCODER_STANDARD_DEVIATION = 0.007;

    private static final double SHOOTER_GEAR_RATIO = 1.5;

    public static final double SHOOTER_REV_TIME = 1.0;

    private SimpleVelocitySystem velocitySystem;
    public boolean fallbackOnEncoder;
    private Timer fallbackTimer;
    
    private HSFalcon master;
    private HSFalcon follower;
    private Encoder shooterEncoder;    
    
    private Shooter() {
        master = new HSFalcon(RobotMap.SHOOTER_MASTER, RobotMap.CANIVORE);
        follower = new HSFalcon(RobotMap.SHOOTER_FOLLOWER, RobotMap.CANIVORE);
        shooterEncoder = new Encoder(RobotMap.SHOOTER_ENCODER_A, RobotMap.SHOOTER_ENCODER_B);
        velocitySystem = new SimpleVelocitySystem(kS, kV, kA, MAX_ERROR, Units.MAX_CONTROL_EFFORT, MODEL_STANDARD_DEVIATION, ENCODER_STANDARD_DEVIATION, RobotMap.LOOP_TIME);
        fallbackOnEncoder = false;
        fallbackTimer = new Timer();
        initMotors();
    }

    public void initMotors() {
        master.configFactoryDefault();
        follower.configFactoryDefault();

        follower.follow(master);

        master.setInverted(MASTER_INVERTED);
        follower.setInverted(FOLLOWER_INVERTED);

        master.configVelocityMeasurementWindow(1);
        master.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_50Ms);

        master.configVoltageCompSaturation(Units.MAX_CONTROL_EFFORT);
        follower.configVoltageCompSaturation(Units.MAX_CONTROL_EFFORT);
    }

    public void setPercentOutput(double speed) {
        master.set(ControlMode.PercentOutput, speed);
    }

    // raw encoder value
    public double getRawVelocity() {
        return shooterEncoder.getRate();
    }

    public double getWheelRPS() {
        // double integratedVel = master.getSelectedSensorVelocity() / Units.FALCON_ENCODER_TICKS / SHOOTER_GEAR_RATIO * 10;
        // double encoderVel = shooterEncoder.getRate() / 1024;
        // if(Math.abs(integratedVel - encoderVel) > 7) {
        //     fallbackOnEncoder = true;
        //     fallbackTimer.start();
        // }
        // else 
        // {
        //     fallbackOnEncoder = false;
        //     fallbackTimer.stop();
        // }
        // return (fallbackOnEncoder) ? integratedVel : encoderVel;
        return shooterEncoder.getRate() / 1024;
    }

    public SimpleVelocitySystem getVelocitySystem() {
        return velocitySystem;
    }

    public void setVelocity(double vel){
        velocitySystem.set(vel);
        velocitySystem.update(getWheelRPS());
        setPercentOutput(velocitySystem.getOutput());
    }

    public Timer getFallbackTimer() {
        return fallbackTimer;
    }

    public HSFalcon getMaster() {
        return master;
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
