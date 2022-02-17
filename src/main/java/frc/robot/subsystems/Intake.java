package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.SimpleVelocitySystem;
import harkerrobolib.wrappers.HSFalcon;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

/**
 * Defines an intake with a motor and a double solenoid
 */
public class Intake extends SubsystemBase {
    private static Intake intake;
    private HSFalcon motor;
    
    private static final boolean MOTOR_INVERT = true;
    
    public static final double MAX_RPS = 100;
    public static final double MIN_RUNNING_RPS = 1;

    private static final double MOTOR_KS = 0;//0.76176;
    private static final double MOTOR_KV = 0.11562;
    private static final double MOTOR_KA = 0.0079187;
    
    private static final double MAX_ERROR = 0.05;
    private static final double MAX_CONTROL_EFFORT = 10;
    private static final double MODEL_STANDARD_DEVIATION = 1;
    private static final double MEASUREMENT_STANDARD_DEVIATION = 0.005;
    private static final double LOOPTIME = 0.02;

    public int state = 0;

    public static final DoubleSolenoid.Value UP = DoubleSolenoid.Value.kReverse;
    public static final DoubleSolenoid.Value DOWN = DoubleSolenoid.Value.kForward;
    
    private DoubleSolenoid doubleSolenoid;
    private SimpleVelocitySystem loop;

    private Intake() {
        motor = new HSFalcon(RobotMap.INTAKE_ID);
        doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.INTAKE_FORWARD, RobotMap.INTAKE_BACKWARD);
        loop = new SimpleVelocitySystem(MOTOR_KS, MOTOR_KV, MOTOR_KA, MAX_ERROR, MAX_CONTROL_EFFORT, 
            MODEL_STANDARD_DEVIATION, MEASUREMENT_STANDARD_DEVIATION, LOOPTIME);
        init();
    }
    

    public void init(){
        motor.configFactoryDefault();
        motor.setInverted(MOTOR_INVERT);
        motor.configVelocityMeasurementWindow(1);
        motor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
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

    public void toggle() {
        if (doubleSolenoid.get() == DoubleSolenoid.Value.kForward) {
            doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
        else {
            doubleSolenoid.set(DoubleSolenoid.Value.kForward);
        }
    }

    public double getCurrentRPS() {
        return 10*motor.getSelectedSensorVelocity() / 2048;
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