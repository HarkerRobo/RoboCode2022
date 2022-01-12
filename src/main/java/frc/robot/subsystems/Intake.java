package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.Conversions.SpeedUnit;
import harkerrobolib.wrappers.HSFalcon;

public class Intake extends SubsystemBase {
    private static Intake instance;
    private HSFalcon rotation;
    private DoubleSolenoid solenoid;
    private static final double ROTATION_P = 0.3;
    private static final double ROTATION_I = 0;
    
    private static final double ROTATION_D = 4;
    private static final double ROTATION_F = 0.05;
    private static final double RAMP_RATE = 0.1;
    private static final double VOLTAGE_COMP = 10;
    private static final double ANGLE_CURRENT_CONTINUOUS = 40;
    private static final double ANGLE_CURRENT_PEAK = 50;
    private static final double ANGLE_CURRENT_PEAK_DUR = 0.1;
    public static final int WHEEL_DIAMETER=2;
    public static final boolean INTAKE_INVERTED=true;

    private static final double CURRENT_DRAW_MIN = 10;
    private static final int JAMMED_VELOCITY = 100;


    private Intake() {
        rotation=new HSFalcon(RobotMap.INTAKE);
        solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.INTAKE_SOLENOID_REVERSE, RobotMap.INTAKE_SOLENOID_FORWARD);
        
        intakeInit();
    }

    public void intakeInit(){
        rotation.configFactoryDefault();
        rotation.setNeutralMode(NeutralMode.Brake);
		rotation.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, ANGLE_CURRENT_CONTINUOUS, ANGLE_CURRENT_PEAK, ANGLE_CURRENT_PEAK_DUR));
		rotation.configVoltageCompSaturation(VOLTAGE_COMP);
		rotation.configForwardSoftLimitEnable(false);

		rotation.config_kP(RobotMap.SLOT_INDEX, ROTATION_P);
		rotation.config_kI(RobotMap.SLOT_INDEX, ROTATION_I);
        rotation.config_kD(RobotMap.SLOT_INDEX, ROTATION_D);
        rotation.config_kF(RobotMap.SLOT_INDEX, ROTATION_F);
        rotation.configClosedloopRamp(RAMP_RATE);

		rotation.selectProfileSlot(RobotMap.SLOT_INDEX, RobotMap.LOOP_INDEX);

        rotation.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, RobotMap.SLOT_INDEX, 100);
        rotation.setInverted(INTAKE_INVERTED);
    }


    public void setPercentOutput(double output){
        rotation.set(ControlMode.PercentOutput, output);
    }

    public void setVelocity(double output){
        rotation.set(ControlMode.Velocity, Conversions.convertSpeed(SpeedUnit.FEET_PER_SECOND,  output, SpeedUnit.ENCODER_UNITS, WHEEL_DIAMETER, 4096));
		
    }

    public boolean isStalling() {
        return rotation.getStatorCurrent() > CURRENT_DRAW_MIN && rotation.getSelectedSensorVelocity() < JAMMED_VELOCITY;
    }

    public HSFalcon getRotation(){
        return rotation;
    }

    public DoubleSolenoid getSolenoid(){
        return solenoid;
    }

    public void invertSolenoid(){
        if(solenoid.get()==Value.kForward){
            solenoid.set(Value.kReverse);
            return;
        }
        solenoid.set(Value.kForward);
    }

    public static Intake getInstance() {
        if (instance == null) {
           instance = new Intake();
        }
        return instance;
     }
}
