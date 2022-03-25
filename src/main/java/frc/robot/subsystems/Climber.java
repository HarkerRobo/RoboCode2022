package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import harkerrobolib.wrappers.HSFalcon;

public class Climber extends SubsystemBase {
    private static Climber climber;

    private static final boolean left_INVERTED = (RobotMap.IS_COMP) ? true : true;
    private static final boolean right_INVERTED = (RobotMap.IS_COMP) ? false : false;

    // private static final double CLIMBER_KP = 2;
    // private static final double CLIMBER_KI = 0;
    // private static final double CLIMBER_KD = 0;

    public static final double MAX_HEIGHT = 119000; //change
    public static final double UP_AND_BACK_HEIGHT = 112000; //change
    public static final double UP_HEIGHT = 117000;
    public static final double STOP_GOING_DOWN_HEIGHT = 65000; //change
    public static final double ON_BAR_HEIGHT = 20000; //change
    public static final double DOWN_HEIGHT = 0; //change
    public static final double ZERO_HEIGHT = 0; //change
    public boolean isZeroed;

    private HSFalcon left;
    private HSFalcon right;
    private DoubleSolenoid piston;
    private DigitalInput leftSwitch;
    private DigitalInput rightSwitch;

    private Climber() {
        left = new HSFalcon(RobotMap.CLIMBER_LEFT, RobotMap.CANIVORE);
        right = new HSFalcon(RobotMap.CLIMBER_RIGHT, RobotMap.CANIVORE);
        piston = new DoubleSolenoid(PneumaticsModuleType.REVPH ,RobotMap.CLIMBER_FORWARD, RobotMap.CLIMBER_BACKWARD);
        leftSwitch = new DigitalInput(RobotMap.CLIMBER_LEFT_LIM_SWITCH);
        rightSwitch = new DigitalInput(RobotMap.CLIMBER_RIGHT_LIM_SWITCH);
        init();
        isZeroed = false;
    }

    public void init() {
        left.configFactoryDefault();
        right.configFactoryDefault();

        right.follow(left);
        left.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, RobotMap.LOOP_INDEX, 20);

        left.setNeutralMode(NeutralMode.Brake);
        right.setNeutralMode(NeutralMode.Brake);
        left.configForwardSoftLimitThreshold(MAX_HEIGHT);
        left.configForwardSoftLimitEnable(false);
        left.setSelectedSensorPosition(0);
        left.configReverseSoftLimitThreshold(2000);
        left.configReverseSoftLimitEnable(false);
        left.overrideLimitSwitchesEnable(false);

        left.configOpenloopRamp(0.6);

        right.configForwardSoftLimitThreshold(MAX_HEIGHT);
        right.configForwardSoftLimitEnable(false);
        right.setSelectedSensorPosition(0);
        right.configReverseSoftLimitThreshold(2000);
        right.configReverseSoftLimitEnable(false);
        right.overrideLimitSwitchesEnable(false);

        right.configOpenloopRamp(0.6);

        // left.config_kP(RobotMap.SLOT_INDEX, CLIMBER_KP);
        // left.config_kI(RobotMap.SLOT_INDEX, CLIMBER_KI);
        // left.config_kD(RobotMap.SLOT_INDEX, CLIMBER_KD);

        left.setInverted(left_INVERTED);
        right.setInverted(right_INVERTED);

        // left.configPeakOutputForward(0.8);
        // left.configPeakOutputReverse(0.8);
    }

    public double getPositionLeft() {
        return left.getSelectedSensorPosition();
    }

    public double getPositionRight() {
        return right.getSelectedSensorPosition();
    }

    public void setClimberOutput(double output) {
        left.set(ControlMode.PercentOutput, output);
    }

    public boolean limitSwitchHit() {
        return !leftSwitch.get() && !rightSwitch.get();
    }

    public boolean leftLimitSwitchHit() {
        return !leftSwitch.get();
    }

    public boolean rightLimitSwitchHit() {
        return !rightSwitch.get();
    }

    // public void setClimberOutputRight(double output) {
    //     right.set(ControlMode.PercentOutput, output);
    // }

    public void toggleClimber() {
        if(piston.get() == DoubleSolenoid.Value.kOff)
            piston.set(DoubleSolenoid.Value.kReverse);
        else piston.toggle();
    }

    public HSFalcon getClimberLeft() {
        return left;
    }

    public HSFalcon getClimberRight() {
        return right;
    }

    public DoubleSolenoid getClimberPiston() {
        return piston;
    }

    public static Climber getInstance() {
        if (climber == null) {
            climber = new Climber();
        }
        return climber;
    }
    
}
