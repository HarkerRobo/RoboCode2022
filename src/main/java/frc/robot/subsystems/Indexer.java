package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private static final boolean INDEXER_SENSOR_IS_0 = (RobotMap.IS_COMP) ? true : true;
    private static final int INDEXER_THRESHOLD = 20;

    private static final double INDEXER_CURRENT_CONTINUOUS = 30;
    private static final double INDEXER_CURRENT_PEAK = 40;
    private static final double INDEXER_CURRENT_PEAK_DUR = 0.2;

    private boolean lastBallWrongColor;
    private double lastBallRecorded;
    private Debouncer debouncer;

    private PicoColorSensor sensor;
    private DigitalInput topSensor;
    private DigitalInput bottomSensor;
    private DigitalInput intakeColorSensor;

    private Indexer() {
        top = new HSFalcon(RobotMap.INDEXER_TOP, RobotMap.CANIVORE);
        bottom = new HSFalcon(RobotMap.INDEXER_BOTTOM, RobotMap.CANIVORE);
        sensor = new PicoColorSensor();
        topSensor = new DigitalInput(RobotMap.INDEXER_TOP_SENSOR);
        bottomSensor = new DigitalInput(RobotMap.INDEXER_BOTTOM_SENSOR);
        lastBallWrongColor = false;
        lastBallRecorded = Timer.getFPGATimestamp();
        debouncer = new Debouncer(1, DebounceType.kFalling);
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
        return !bottomSensor.get();
    }

    public boolean topOccupied() {
        return !topSensor.get();
    }

    public boolean intakeHasWrongColor() {

        // if(getIndexerProximity() >= INDEXER_THRESHOLD)
        // {
        //     lastBallWrongColor = intakeHasWrongColorNotDebounced();   
        //     lastBallRecorded = Timer.getFPGATimestamp();
        // }
        // if(Timer.getFPGATimestamp() - lastBallRecorded >= 0.4)
        //     lastBallWrongColor = false;
        // return lastBallWrongColor;
        SmartDashboard.putNumber("intake not debounced", intakeHasWrongColorNotDebounced() ? 1 : 0);
        SmartDashboard.putNumber("intake debounced", debouncer.calculate(intakeHasWrongColorNotDebounced()) ? 1 : 0);
        return false;
        // return debouncer.calculate(intakeHasWrongColorNotDebounced());
    }

    public boolean intakeHasWrongColorNotDebounced() {
        if(getIndexerProximity() >= INDEXER_THRESHOLD) 
            return false;
        if(DriverStation.getAlliance() == DriverStation.Alliance.Red)
            return getColor().blue > getColor().red;
        else 
            return getColor().red > getColor().blue;
    }

    public RawColor getColor() {
        if(INDEXER_SENSOR_IS_0)
            return sensor.getRawColor0();
        return sensor.getRawColor1();
    }

    public int getIndexerProximity() {
        if(INDEXER_SENSOR_IS_0)
            return sensor.getProximity0();
        return sensor.getProximity1();
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
