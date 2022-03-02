package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Units;
import frc.robot.util.InterpolatedTreeMap;
import harkerrobolib.wrappers.HSFalcon;

public class Hood extends SubsystemBase{
    private static final boolean HOOD_INVERTED = (RobotMap.IS_COMP) ? true : false;
    public static boolean isZeroed = false;
    
    public static final double HOOD_GEAR_RATIO = 180;

    public static final double HOOD_RANGE_DEGREES = 23;

    private static final double HOOD_KP = 1.0; 
    private static final double HOOD_KI = 0.8;
    private static final double HOOD_KD = 0.018539; 
    private static final double HOOD_IZONE = 100000; 
    private static final double HOOD_KS = 0.63651; 
    private static final double HOOD_KV = 0.051961;

    private ProfiledPIDController hoodController;
    private InterpolatedTreeMap referencePoints;
    private SimpleMotorFeedforward feedforward;

    private static final double HOOD_CURRENT_CONTINUOUS = 10;
    private static final double HOOD_CURRENT_PEAK = 10;
    private static final double HOOD_CURRENT_PEAK_DUR = 0.05;

    private static InterpolatedTreeMap refPoints;

    private HSFalcon hood;

    private static Hood instance;
    
    private Hood() {
        hood = new HSFalcon(RobotMap.HOOD, RobotMap.CANIVORE);
        initMotors();
        refPoints = new InterpolatedTreeMap();
        refPoints.put(0.88, 10.0);
        refPoints.put(1.12, 14.0);
        refPoints.put(1.49, 16.0);
        refPoints.put(1.89, 18.0);
        refPoints.put(2.27, 21.0);
        refPoints.put(2.67, 23.0);

        hoodController = new ProfiledPIDController(HOOD_KP, HOOD_KI, HOOD_KD, new Constraints(30, 15));
        hoodController.setIntegratorRange(-HOOD_IZONE, HOOD_IZONE);
        feedforward = new SimpleMotorFeedforward(HOOD_KS, HOOD_KV);
    }
    
    private void initMotors() {
        hood.configFactoryDefault();
        hood.setInverted(HOOD_INVERTED);
        hood.configVoltageCompSaturation(Units.MAX_CONTROL_EFFORT);
        hood.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, HOOD_CURRENT_CONTINUOUS, HOOD_CURRENT_PEAK, HOOD_CURRENT_PEAK_DUR));
        // hood.configPeakOutputForward(0.3);
        // hood.configPeakOutputReverse(-0.3);
    }

    public double getHoodPos(){
        return hood.getSelectedSensorPosition() * Units.FALCON_ENCODER_TO_DEGREE / HOOD_GEAR_RATIO;
    }

    public ProfiledPIDController getController(){
        return hoodController;
    }

    public HSFalcon getHood() {
        return hood;
    }
    
    public static Hood getInstance() {
        if (instance == null) {
            instance = new Hood();
        }
        return instance;
    }
}
