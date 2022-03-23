package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Units;
import frc.robot.util.SimpleVelocitySystem;
import harkerrobolib.wrappers.HSFalcon;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

/**
 * Defines an intake with a motor and a double solenoid
 */
public class Intake extends SubsystemBase {
    private static Intake intake;
    private HSFalcon motor;
    
    private static final boolean MOTOR_INVERT = (RobotMap.IS_COMP) ? true : true;
    
    public static final double MAX_RPS = 60;
    public static final double MIN_RUNNING_RPS = 1;
    public static final double INTAKE_GEAR_RATIO = 0.6;

    private static final double MOTOR_KS = 0.69616;
    private static final double MOTOR_KV = 0.18026;
    private static final double MOTOR_KA = 0.0083494;

    private static final double CONTINUOUS_CURRENT_LIMIT = 30;
    private static final double PEAK_CURRENT = 40;
    private static final double PEAK_DUR = 0.1;
    
    private static final double MAX_ERROR = 0.05;
    private static final double MODEL_STANDARD_DEVIATION = 1;
    private static final double MEASUREMENT_STANDARD_DEVIATION = 0.005;
    private static final double LOOPTIME = 0.02;

    public int state = 0;
    
    private DoubleSolenoid doubleSolenoid;
    private SimpleVelocitySystem loop;

    private Intake() {
        motor = new HSFalcon(RobotMap.INTAKE_ID);
        doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.INTAKE_FORWARD, RobotMap.INTAKE_BACKWARD);
        loop = new SimpleVelocitySystem(MOTOR_KS, MOTOR_KV, MOTOR_KA, MAX_ERROR, Units.MAX_CONTROL_EFFORT, 
            MODEL_STANDARD_DEVIATION, MEASUREMENT_STANDARD_DEVIATION, LOOPTIME);
        init();
    }
    

    public void init(){
        motor.configFactoryDefault();
        motor.setInverted(MOTOR_INVERT);
        motor.configVelocityMeasurementWindow(1);
        motor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
        motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, CONTINUOUS_CURRENT_LIMIT, PEAK_CURRENT, PEAK_DUR));
    }
   
    public void setPercentOutput(double output) {
        if (Math.abs(output) > 0.8)
            output = 0.8 * Math.signum(output);
         motor.set(ControlMode.PercentOutput, output);
    }

    public void setVelocity(double vel){
        loop.set(vel);
        loop.update(getCurrentRPS());
        setPercentOutput(loop.getOutput());
        // state = (int)Math.signum(vel);
        // setPercentOutput(vel);
    }

    public double getCurrentRPS() {
        return 10*motor.getSelectedSensorVelocity() / 2048 * INTAKE_GEAR_RATIO;
    }

    public SimpleVelocitySystem getLoop(){
        return loop;
    }

    public HSFalcon getMotor(){
        return motor;
    }

    public DoubleSolenoid getSolenoid() {
        return doubleSolenoid;
    }

    public static Intake getInstance() {
        if (intake == null) {
            intake = new Intake();
        }
        return intake;
    }
}