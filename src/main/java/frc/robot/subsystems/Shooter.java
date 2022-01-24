package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Units;
import frc.robot.util.SimpleVelocitySystem;
import harkerrobolib.wrappers.HSFalcon;

public class Shooter extends SubsystemBase {
    private static Shooter shooter;

    private static final boolean MASTER_INVERTED = true;
    private static final boolean FOLLOWER_INVERTED = false;
    
    
    public static final double kS = 0.70457;    
    public static final double kV = 0.33337;
    public static final double kA = 0.060663;
    
    private static final double MAX_CONTROL_EFFORT = 12; // volts 
    private static final double MODEL_STANDARD_DEVIATION = 3;
    private static final double ENCODER_STANDARD_DEVIATION = 0.1;

    private SimpleVelocitySystem velocitySystem;
    
    private HSFalcon master;
    private HSFalcon follower;
    
    private Shooter() {
        master = new HSFalcon(RobotMap.SHOOTER_MASTER);
        follower = new HSFalcon(RobotMap.SHOOTER_FOLLOWER);
    
        initMotors();
        velocitySystem = new SimpleVelocitySystem(kS, kV, kA, MAX_CONTROL_EFFORT, MODEL_STANDARD_DEVIATION, ENCODER_STANDARD_DEVIATION, RobotMap.LOOP_TIME);
    }

    public void initMotors() {
        follower.follow(master);

        master.setInverted(MASTER_INVERTED);
        follower.setInverted(FOLLOWER_INVERTED);

        master.configVelocityMeasurementWindow(1);
        master.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);

        master.configVoltageCompSaturation(MAX_CONTROL_EFFORT);
        follower.configVoltageCompSaturation(MAX_CONTROL_EFFORT);
    }

    public void setPercentOutput(double speed) {
        master.set(ControlMode.PercentOutput, speed);
    }

    // m/s, raw encoder value
    public double getRawVelocity() {
        return Shooter.getInstance().getMaster().getSelectedSensorVelocity() * 10 / Units.TICKS_PER_REVOLUTION * Units.FLYWHEEL_ROT_TO_METER;
    }

    public SimpleVelocitySystem getVelocitySystem() {
        return velocitySystem;
    }

    public HSFalcon getMaster() {
        return master;
    }

    public static Shooter getInstance() {
        if (shooter == null) {
            shooter = new Shooter();
        }
        return shooter;
    }
}
