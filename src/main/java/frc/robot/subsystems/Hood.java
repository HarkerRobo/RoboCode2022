package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Units;
import harkerrobolib.wrappers.HSFalcon;

public class Hood extends SubsystemBase{
    private static final boolean HOOD_INVERTED = (RobotMap.IS_COMP) ? true : true;
    public static boolean isZeroed = false;
    
    public static final double HOOD_GEAR_RATIO = 180;

    public static final double HOOD_RANGE_DEGREES = 23;

    private static final double HOOD_CURRENT_CONTINUOUS = 10;
    private static final double HOOD_CURRENT_PEAK = 10;
    private static final double HOOD_CURRENT_PEAK_DUR = 0.05;

    private static final double HOOD_LIMIT = RobotMap.IS_COMP ? 23552 : 22000;

    private HSFalcon hood;

    private static Hood instance;
    
    private Hood() {
        hood = new HSFalcon(RobotMap.HOOD, RobotMap.CANIVORE);
        initMotors();
    }
    
    private void initMotors() {
        hood.configFactoryDefault();
        hood.setInverted(HOOD_INVERTED);
        hood.configVoltageCompSaturation(Units.MAX_CONTROL_EFFORT);
        hood.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, HOOD_CURRENT_CONTINUOUS, HOOD_CURRENT_PEAK, HOOD_CURRENT_PEAK_DUR));
        hood.configForwardSoftLimitThreshold(HOOD_LIMIT);
        hood.configReverseSoftLimitThreshold(0);
        hood.configForwardSoftLimitEnable(true);
        hood.setNeutralMode(NeutralMode.Brake);
        
        hood.configPeakOutputForward(0.3);
        hood.configPeakOutputReverse(-0.3);
        hood.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        hood.configVelocityMeasurementWindow(8);
    }

    public double getHoodPos(){
        return hood.getSelectedSensorPosition() * Units.FALCON_ENCODER_TO_DEGREE / HOOD_GEAR_RATIO;
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
