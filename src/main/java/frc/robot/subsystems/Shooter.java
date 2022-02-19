package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Units;
import frc.robot.util.InterpolatedTreeMap;
import frc.robot.util.SimpleVelocitySystem;
import harkerrobolib.wrappers.HSFalcon;

/**
 * Specifies a two-motor shooter with a linear quadratic regulator and a kalman filter
 */
public class Shooter extends SubsystemBase {
    private static Shooter shooter;

    private static final boolean MASTER_INVERTED = true;
    private static final boolean FOLLOWER_INVERTED = false;
    
    public static final double kS = 0;//0.67951;    
    public static final double kV = 0.15808;
    public static final double kA = 0.015239;

    public static final double HOOD_STALLING_CURRENT = 10;
    
    private static final double MAX_ERROR = 0.1; // volts 
    private static final double MODEL_STANDARD_DEVIATION = 0.00625;
    private static final double ENCODER_STANDARD_DEVIATION = 0.00025;

    public static final double SHOOTER_REV_TIME = 1.0;

    private SimpleVelocitySystem velocitySystem;
    public static InterpolatedTreeMap referencePoints;
    
    private HSFalcon master;
    private HSFalcon follower;
    private Encoder shooterEncoder;    
    
    private Shooter() {
        master = new HSFalcon(RobotMap.SHOOTER_MASTER);
        follower = new HSFalcon(RobotMap.SHOOTER_FOLLOWER);
        shooterEncoder = new Encoder(RobotMap.SHOOTER_ENCODER_A, RobotMap.SHOOTER_ENCODER_B);
        initMotors();
        velocitySystem = new SimpleVelocitySystem(kS, kV, kA, MAX_ERROR, Units.MAX_CONTROL_EFFORT, MODEL_STANDARD_DEVIATION, ENCODER_STANDARD_DEVIATION, RobotMap.LOOP_TIME);
        referencePoints = new InterpolatedTreeMap();
        referencePoints.put(0.17, 27.0);
        referencePoints.put(0.7, 30.0);
        referencePoints.put(1.0, 31.0);
        referencePoints.put(1.3, 31.0);
        referencePoints.put(1.75, 28.0);
        referencePoints.put(1.98, 29.0);
        referencePoints.put(2.38, 31.0);
        referencePoints.put(2.8, 33.0);
        referencePoints.put(3.24, 33.0);
        referencePoints.put(3.7, 39.0);
        referencePoints.put(4.1, 43.0);
    }

    public void initMotors() {
        master.configFactoryDefault();
        follower.configFactoryDefault();

        follower.follow(master);

        master.setInverted(MASTER_INVERTED);
        follower.setInverted(FOLLOWER_INVERTED);

        master.configVelocityMeasurementWindow(1);
        master.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);

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
        return shooterEncoder.getRate() / 1024;
    }

    public SimpleVelocitySystem getVelocitySystem() {
        return velocitySystem;
    }


    public void setVelocity(double vel){
        velocitySystem.set(vel);
        velocitySystem.update(getWheelRPS());
        // setPercentOutput(vel);
        setPercentOutput(velocitySystem.getOutput());
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
