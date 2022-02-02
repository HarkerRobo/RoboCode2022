package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.SimpleVelocitySystem;
import harkerrobolib.wrappers.HSFalcon;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

public class Intake extends SubsystemBase {
    private static Intake intake;
    private HSFalcon motor;
    
    private static final boolean MOTOR_INVERT = true;
    
    private static final double MOTOR_KS = 0.3;
    private static final double MOTOR_KV = 0.5;
    private static final double MOTOR_KA = 0.02;
    
    private static final double MAX_CONTROL_EFFORT = 12;
    private static final double MODEL_STANDARD_DEVIATION = 10;
    private static final double MEASUREMENT_STANDARD_DEVIATION = 5;
    private static final double LOOPTIME = 0.02;

    public static final DoubleSolenoid.Value UP = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value DOWN = DoubleSolenoid.Value.kReverse; 
    
    private DoubleSolenoid doubleSolenoid;
    private SimpleVelocitySystem loop;

    private Intake() {
        motor = new HSFalcon(RobotMap.INTAKE_ID);
        doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.INTAKE_FORWARD, RobotMap.INTAKE_BACKWARD);
        loop = new SimpleVelocitySystem(MOTOR_KS, MOTOR_KV, MOTOR_KA, MAX_CONTROL_EFFORT, 
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
        motor.set(ControlMode.PercentOutput, output);
    }

    public void setVelocity(double vel){
        loop.set(vel);
        loop.update(getCurrentRPM());
        // setPercentOutput(loop.getOutput());
        setPercentOutput(vel);
    }

    public void toggle() {
        if (doubleSolenoid.get() == DoubleSolenoid.Value.kForward) {
            doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
        else {
            doubleSolenoid.set(DoubleSolenoid.Value.kForward);
        }
    }

    public double getCurrentRPM() {
        return 600*motor.getSelectedSensorVelocity() / 2048;
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