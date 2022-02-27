// package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotMap;
// import harkerrobolib.wrappers.HSFalcon;

// public class Climber extends SubsystemBase {
//     private static Climber climber;

//     private static final boolean MASTER_INVERTED = (RobotMap.IS_COMP) ? false : false;
//     private static final boolean FOLLOWER_INVERTED = (RobotMap.IS_COMP) ? false : false;

//     private static final double CLIMBER_KP = 2;
//     private static final double CLIMBER_KI = 0;
//     private static final double CLIMBER_KD = 0;

//     public static final double MAX_HEIGHT = 100000; //change
//     public static final double DOWN_HEIGHT = 10000; //change
//     public static final double ZERO_HEIGHT = 0; //change

//     private HSFalcon master;
//     private HSFalcon follower;
//     private DoubleSolenoid piston;

//     private Climber() {
//         master = new HSFalcon(RobotMap.CLIMBER_MASTER, RobotMap.CANIVORE);
//         follower = new HSFalcon(RobotMap.CLIMBER_FOLLOWER, RobotMap.CANIVORE);
//         piston = new DoubleSolenoid(PneumaticsModuleType.REVPH ,RobotMap.CLIMBER_FORWARD, RobotMap.CLIMBER_BACKWARD);
//     }

//     public void init() {
//         master.configFactoryDefault();
//         follower.configFactoryDefault();

//         master.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, RobotMap.LOOP_INDEX, 20);

//         follower.follow(master);

//         master.configForwardSoftLimitEnable(true);
//         master.configForwardSoftLimitThreshold(MAX_HEIGHT);
//         master.setSelectedSensorPosition(0);
//         follower.configReverseSoftLimitEnable(true);
//         follower.configReverseSoftLimitThreshold(0);

//         master.configOpenloopRamp(0.1);
//         master.config_kP(RobotMap.SLOT_INDEX, CLIMBER_KP);
//         master.config_kI(RobotMap.SLOT_INDEX, CLIMBER_KI);
//         master.config_kD(RobotMap.SLOT_INDEX, CLIMBER_KD);

//         master.setInverted(MASTER_INVERTED);
//         follower.setInverted(FOLLOWER_INVERTED);

//         // master.configPeakOutputForward(0.8);
//         // master.configPeakOutputReverse(0.8);
//     }

//     public void setClimberPosition(double pos) {
//         pos = MAX_HEIGHT * (pos / Math.abs(pos));
//         master.set(ControlMode.Position, pos);
//     }

//     public HSFalcon getClimberMaster() {
//         return master;
//     }

//     public HSFalcon getClimberFollower() {
//         return follower;
//     }

//     public DoubleSolenoid getClimberPiston() {
//         return piston;
//     }

//     public static Climber getInstance() {
//         if (climber == null) {
//             climber = new Climber();
//         }
//         return climber;
//     }
    
// }
