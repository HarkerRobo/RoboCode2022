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

    private LinearSystem<N1, N1, N1> system = LinearSystemId.identifyVelocitySystem(kV, kA);
    private LinearQuadraticRegulator<N1, N1, N1> regulator = new LinearQuadraticRegulator<N1,N1,N1>(system, VecBuilder.fill(0.05), VecBuilder.fill(MAX_CONTROL_EFFORT), RobotMap.LOOP_TIME);
    private KalmanFilter<N1, N1, N1> filter = new KalmanFilter<N1, N1, N1>(Nat.N1(), Nat.N1(), system, VecBuilder.fill(MODEL_STANDARD_DEVIATION), VecBuilder.fill(ENCODER_STANDARD_DEVIATION), RobotMap.LOOP_TIME);
    private LinearSystemLoop<N1, N1, N1> loop = new LinearSystemLoop<N1, N1, N1>(system, regulator, filter, MAX_CONTROL_EFFORT, RobotMap.LOOP_TIME);
    
    private HSFalcon master;
    private HSFalcon follower;
    
    private Shooter() {
        master = new HSFalcon(RobotMap.SHOOTER_MASTER);
        follower = new HSFalcon(RobotMap.SHOOTER_FOLLOWER);
    
        initMotors();
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

    public void setControllerTarget(double vel) {
        loop.setNextR(VecBuilder.fill(vel)); // set output
    }
    
    public void updateController() {
        loop.correct(VecBuilder.fill(Shooter.getInstance().getRawVelocity()));
        loop.predict(0.02);
    }

    // percent output
    public double getControllerOutput() {
        return (loop.getU(0) + kS) / MAX_CONTROL_EFFORT;
    }

    // m/s, raw encoder value
    public double getRawVelocity() {
        return Shooter.getInstance().getMaster().getSelectedSensorVelocity() * 10 / Units.TICKS_PER_REVOLUTION * Units.FLYWHEEL_ROT_TO_METER;
    }

    // m/s, kalman filtered
    public double getFilteredVelocity() {
        return loop.getXHat(0);
    }
    
    public KalmanFilter<N1, N1, N1> getFilter() {
        return filter;
    }

    public LinearSystemLoop<N1, N1, N1> getLoop() {
        return loop;
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
