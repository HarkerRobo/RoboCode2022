package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import harkerrobolib.wrappers.HSFalcon;

public class Climber extends SubsystemBase {
    private static Climber climber;

    private static final boolean MASTER_INVERTED = (RobotMap.IS_COMP) ? false : false;
    private static final boolean FOLLOWER_INVERTED = (RobotMap.IS_COMP) ? false : true;

    // private static final double CLIMBER_KP = 2;
    // private static final double CLIMBER_KI = 0;
    // private static final double CLIMBER_KD = 0;

    public static final double MAX_HEIGHT = 112000; //change
    public static final double DOWN_HEIGHT = 10000; //change
    public static final double ZERO_HEIGHT = 0; //change
    public boolean isZeroed;

    private HSFalcon master;
    private HSFalcon follower;
    private DoubleSolenoid piston;

    private Climber() {
        master = new HSFalcon(RobotMap.CLIMBER_MASTER, RobotMap.CANIVORE);
        follower = new HSFalcon(RobotMap.CLIMBER_FOLLOWER, RobotMap.CANIVORE);
        piston = new DoubleSolenoid(PneumaticsModuleType.REVPH ,RobotMap.CLIMBER_FORWARD, RobotMap.CLIMBER_BACKWARD);
        init();
        isZeroed = false;
    }

    public void init() {
        master.configFactoryDefault();
        follower.configFactoryDefault();

        master.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, RobotMap.LOOP_INDEX, 20);

        follower.follow(master);
        master.setNeutralMode(NeutralMode.Brake);
        follower.setNeutralMode(NeutralMode.Brake);
        master.configForwardSoftLimitThreshold(MAX_HEIGHT);
        master.configForwardSoftLimitEnable(true);
        master.setSelectedSensorPosition(0);
        master.configReverseSoftLimitThreshold(2000);
        master.configReverseSoftLimitEnable(false);

        master.configOpenloopRamp(0.3);
        // master.config_kP(RobotMap.SLOT_INDEX, CLIMBER_KP);
        // master.config_kI(RobotMap.SLOT_INDEX, CLIMBER_KI);
        // master.config_kD(RobotMap.SLOT_INDEX, CLIMBER_KD);

        master.setInverted(MASTER_INVERTED);
        follower.setInverted(FOLLOWER_INVERTED);

        // master.configPeakOutputForward(0.8);
        // master.configPeakOutputReverse(0.8);
    }

    public double getPosition() {
        return master.getSelectedSensorPosition();
    }

    public void setClimberOutput(double output) {
        master.set(ControlMode.PercentOutput, output);
    }

    public void toggleClimber() {
        if(piston.get() == DoubleSolenoid.Value.kOff)
            piston.set(DoubleSolenoid.Value.kForward);
        else piston.toggle();
    }

    public HSFalcon getClimberMaster() {
        return master;
    }

    public HSFalcon getClimberFollower() {
        return follower;
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
