package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.PicoColorSensor;
import frc.robot.util.PicoColorSensor.RawColor;
import harkerrobolib.wrappers.HSFalcon;

public class Indexer extends SubsystemBase {

    private static Indexer indexer;

    private HSFalcon top;
    private HSFalcon bottom;

    private static final boolean TOP_INVERT = true;
    private static final boolean BOTTOM_INVERT = false;

    private static final boolean BOTTOM_SENSOR_IS_0 = true;
    private static final int[][] RED_COLOR_RANGE = {{170,255},{0,0},{0,0}};
    private static final int[][] BLUE_COLOR_RANGE = {{0,0},{0,0},{170,255}};

    private PicoColorSensor sensor;

    private Indexer() {
        top = new HSFalcon(RobotMap.INDEXER_TOP);
        bottom = new HSFalcon(RobotMap.INDEXER_BOTTOM);
        sensor = new PicoColorSensor();
        init();
    }

    private void init() {
        top.configFactoryDefault();
        top.setInverted(TOP_INVERT);
        bottom.configFactoryDefault();
        bottom.setInverted(BOTTOM_INVERT);
        top.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        top.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
        bottom.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        bottom.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
    }

    public boolean bottomOccupied() {
        if(BOTTOM_SENSOR_IS_0)
            return sensor.getProximity0() > 500;
        return sensor.getProximity1() > 500;
    }

    public boolean topOccupied() {
        if(BOTTOM_SENSOR_IS_0)
            return sensor.getProximity1() > 500;
        return sensor.getProximity0() > 500;
    }

    public boolean bottomHasRed() {
        return isRed(getColor());
    }

    public boolean bottomHasBlue() {
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
        if(col.red < RED_COLOR_RANGE[0][0] || col.red > RED_COLOR_RANGE[0][1])
            return false;
        if(col.blue < RED_COLOR_RANGE[1][0] || col.blue > RED_COLOR_RANGE[1][1])
            return false;
        if(col.green < RED_COLOR_RANGE[2][0] || col.green > RED_COLOR_RANGE[2][1])
            return false;
        return true;
    }

    private boolean isBlue(RawColor col) {
        if(col.red < BLUE_COLOR_RANGE[0][0] || col.red > BLUE_COLOR_RANGE[0][1])
            return false;
        if(col.blue < BLUE_COLOR_RANGE[1][0] || col.blue > BLUE_COLOR_RANGE[1][1])
            return false;
        if(col.green < BLUE_COLOR_RANGE[2][0] || col.green > BLUE_COLOR_RANGE[2][1])
            return false;
        return true;
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
