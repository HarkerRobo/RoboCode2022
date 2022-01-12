package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import harkerrobolib.wrappers.HSTalon;

public class Indexer extends SubsystemBase {
    private static Indexer instance;

    private  HSTalon linear;
    private VictorSPX agitator;
    private DoubleSolenoid blocker;
    private DigitalInput shooterSensor;
    private DigitalInput linearSensor;


    private static final double VOLTAGE_COMP = 10;
    private static final double CURRENT_CONTINUOUS =30;
    private static final double CURRENT_PEAK = 40;
    private static final double CURRENT_PEAK_DUR = 0.1;

    public static final Value BLOCKER_OPEN = Value.kReverse;
    public static final Value BLOCKER_CLOSED = Value.kForward;

    Indexer(){
        linear=new HSTalon(RobotMap.INDEXER_LINEAR);
        agitator=new VictorSPX(RobotMap.INDEXER_AGITATOR);
        blocker= new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.INDEXER_BLOCKER_FORWARD,RobotMap.INDEXER_BLOCKER_REVERSE);

        shooterSensor=new DigitalInput(RobotMap.SHOOTER_SENSOR);
        linearSensor=new DigitalInput(RobotMap.LINEAR_SENSOR);

        initLinear();
        initAgitator();
    }

    private HSTalon getLinear(){
        return linear;
    }
    public VictorSPX getAlternator(){
        return agitator;
    }
    public DoubleSolenoid getSolenoid(){
        return blocker;
    }

    public boolean shooterSensorBlocked(){
        return !shooterSensor.get();
    }
    public boolean linearSensorBlocked(){
        return !linearSensor.get();
    }

    public void initLinear(){
        linear.configFactoryDefault();
        linear.setNeutralMode(NeutralMode.Brake);
		linear.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, CURRENT_CONTINUOUS, CURRENT_PEAK, CURRENT_PEAK_DUR));
		linear.configVoltageCompSaturation(VOLTAGE_COMP);
		linear.configForwardSoftLimitEnable(false);
    }

    public void initAgitator(){
        agitator.configFactoryDefault();
        agitator.setNeutralMode(NeutralMode.Brake);
		agitator.configVoltageCompSaturation(VOLTAGE_COMP);
		agitator.configForwardSoftLimitEnable(false);
    }

    public void setLinearPercentOutput(double output){
        linear.set(ControlMode.PercentOutput, output);
    }

    public void setAgitatorPercentOutput(double output){
        agitator.set(ControlMode.PercentOutput, output);
    }

    public static Indexer getInstance() {
        if (instance == null) {
           instance = new Indexer();
        }
        return instance;
     }
}
