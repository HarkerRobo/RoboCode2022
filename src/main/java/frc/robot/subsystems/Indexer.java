package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.PicoColorSensor;
import frc.robot.util.PicoColorSensor.RawColor;
import harkerrobolib.wrappers.HSFalcon;

public class Indexer extends SubsystemBase {

    private static Indexer indexer;

    private HSFalcon top;
    private HSFalcon bottom;

    private static final boolean TOP_INVERT = (RobotMap.IS_COMP) ? true : true;
    private static final boolean BOTTOM_INVERT = (RobotMap.IS_COMP) ? false : false;

    private static final boolean BOTTOM_SENSOR_IS_0 = (RobotMap.IS_COMP) ? true : false;
    private static final int TOP_THRESHOLD = 300;
    private static final int BOTTOM_THRESHOLD = 300;
    private static final ColorMatch matcher = new ColorMatch();

    private static final double INDEXER_CURRENT_CONTINUOUS = 30;
    private static final double INDEXER_CURRENT_PEAK = 30;
    private static final double INDEXER_CURRENT_PEAK_DUR = 0.05;

    private PicoColorSensor sensor;

    private Indexer() {
        top = new HSFalcon(RobotMap.INDEXER_TOP, RobotMap.CANIVORE);
        bottom = new HSFalcon(RobotMap.INDEXER_BOTTOM, RobotMap.CANIVORE);
        sensor = new PicoColorSensor();
        matcher.addColorMatch(new Color(1, 0, 0));
        matcher.addColorMatch(new Color(0, 0, 1));
        matcher.setConfidenceThreshold(0.05);
        init();
    }

    private void init() {
        top.configFactoryDefault();
        top.setInverted(TOP_INVERT);
        bottom.configFactoryDefault();
        bottom.setInverted(BOTTOM_INVERT);

        top.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, INDEXER_CURRENT_CONTINUOUS, INDEXER_CURRENT_PEAK, INDEXER_CURRENT_PEAK_DUR));
        bottom.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, INDEXER_CURRENT_CONTINUOUS, INDEXER_CURRENT_PEAK, INDEXER_CURRENT_PEAK_DUR));

        top.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        top.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
        bottom.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        bottom.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
    }

    public boolean bottomOccupied() {
        if(BOTTOM_SENSOR_IS_0)
            return sensor.getProximity0() > BOTTOM_THRESHOLD;
        return sensor.getProximity1() > BOTTOM_THRESHOLD;
    }

    public boolean topOccupied() {
        if(BOTTOM_SENSOR_IS_0)
            return sensor.getProximity1() > TOP_THRESHOLD;
        return sensor.getProximity0() > TOP_THRESHOLD;
    }

    public boolean bottomisWrongColor() {
        if(DriverStation.getAlliance() == DriverStation.Alliance.Red){
            return isBlue(getColor());
        }
        return isRed(getColor());
    }

    public RawColor getColor() {
        if(BOTTOM_SENSOR_IS_0)
            return sensor.getRawColor0();
        return sensor.getRawColor1();
    }

    public int getProximity() {
        if(BOTTOM_SENSOR_IS_0)
        return sensor.getProximity0();
    return sensor.getProximity1();
    }

    private boolean isRed(RawColor col) {
        ColorMatchResult match = matcher.matchColor(new Color(col.red/255, col.green/255, col.blue/255));
        if(match == null) return false;
        return match.color.red == 1.0;
    }

    private boolean isBlue(RawColor col) {
        ColorMatchResult match = matcher.matchColor(new Color(col.red/255, col.green/255, col.blue/255));
        if(match == null) return false;
        return match.color.blue == 1.0;
    }

    public void setPercentOutputBottom(double output) {
        bottom.set(ControlMode.PercentOutput, output);
    }

    public void setPercentOutputTop(double output) {
        top.set(ControlMode.PercentOutput, output);
    }

    public static Indexer getInstance() {
        if(indexer == null)
            indexer = new Indexer();
        return indexer;
    }
}
