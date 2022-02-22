package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Units;
import harkerrobolib.wrappers.HSFalcon;

public class Hood extends SubsystemBase{
    private static final boolean HOOD_INVERTED = true;
    public static boolean isZeroed = false;
    
    public static final double HOOD_GEAR_RATIO = 7.5; //change

    private static double hoodMaxPos = 1.77;
    private static double hoodEncoderOffset = 0;
    private static final double HOOD_RANGE_DEGREES = 23;

    private static final double HOOD_CURRENT_CONTINUOUS = 10;
    private static final double HOOD_CURRENT_PEAK = 10;
    private static final double HOOD_CURRENT_PEAK_DUR = 0.05;

    private HSFalcon hood;
    private Counter hoodEncoder;

    private static Hood instance;
    
    private Hood() {
        hood = new HSFalcon(RobotMap.HOOD);
        hoodEncoder = new Counter(RobotMap.HOOD_ENCODER);
        hoodEncoder.setSemiPeriodMode(true);
        initMotors();
    }
    
    private void initMotors() {
        hood.configFactoryDefault();
        hood.setInverted(HOOD_INVERTED);
        hood.configVoltageCompSaturation(Units.MAX_CONTROL_EFFORT);
        hood.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, HOOD_CURRENT_CONTINUOUS, HOOD_CURRENT_PEAK, HOOD_CURRENT_PEAK_DUR));
        // hood.configPeakOutputForward(0.3);
        // hood.configPeakOutputReverse(-0.3);
    }

    public double getHoodEncoderPos() {
        return 1 - (1e6 * hoodEncoder.getPeriod() / Units.MAG_CODER_ENCODER_TICKS);
    }

    public void setHoodOffset() {
        hoodEncoderOffset = getHoodEncoderPos();
    }

    public void setHoodMax() {
        hoodMaxPos = getHoodPos();
    }

    public double getHoodPos(){
        double falconPos = hood.getSelectedSensorPosition() / Units.FALCON_ENCODER_TICKS / HOOD_GEAR_RATIO;
        double magPos = getHoodEncoderPos() - hoodEncoderOffset;
        while(falconPos - magPos > 0.5)
            magPos++;
        return magPos;
    }

    public double getHoodPosDegrees() {
        return getHoodPos()/hoodMaxPos * HOOD_RANGE_DEGREES;
    }

    public double getMaxHoodPos() {
        return hoodMaxPos;
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
