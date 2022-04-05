package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import harkerrobolib.wrappers.HSFalcon;

public class Indexer extends SubsystemBase {

    private static Indexer indexer;

    private HSFalcon top;
    private HSFalcon bottom;

    private static final boolean TOP_INVERT = (RobotMap.IS_COMP) ? true : true;
    private static final boolean BOTTOM_INVERT = (RobotMap.IS_COMP) ? false : false;

    private static final double INDEXER_CURRENT_CONTINUOUS = 30;
    private static final double INDEXER_CURRENT_PEAK = 60;
    private static final double INDEXER_CURRENT_PEAK_DUR = 0.2;

    private DigitalInput topSensor;
    private DigitalInput bottomSensor;
    // private DigitalInput indexerRed;
    // private DigitalInput indexerBlue;
    private Counter indexerRed;
    private Counter indexerBlue;

    private Debouncer wrong;
    private Debouncer correct;

    private Indexer() {
        top = new HSFalcon(RobotMap.INDEXER_TOP, RobotMap.CANIVORE);
        bottom = new HSFalcon(RobotMap.INDEXER_BOTTOM, RobotMap.CANIVORE);
        topSensor = new DigitalInput(RobotMap.INDEXER_TOP_SENSOR);
        bottomSensor = new DigitalInput(RobotMap.INDEXER_BOTTOM_SENSOR);
        // indexerRed = new DigitalInput(RobotMap.INDEXER_RED);
        // indexerBlue = new DigitalInput(RobotMap.INDEXER_BLUE);
        indexerRed = new Counter(RobotMap.INDEXER_RED);
        indexerBlue = new Counter(RobotMap.INDEXER_BLUE);
        indexerRed.setSemiPeriodMode(true);
        indexerBlue.setSemiPeriodMode(true);
        wrong = new Debouncer(0.7, Debouncer.DebounceType.kFalling);
        correct = new Debouncer(0.7, Debouncer.DebounceType.kFalling);
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
        return false;
        // if(DriverStation.getAlliance() == Alliance.Blue)
        //     return wrong.calculate(indexerRed.get());
        // return wrong.calculate(indexerBlue.get());
    }

    public boolean intakeHasRightColor() {
        return true;
        // if(DriverStation.getAlliance() == Alliance.Red)
        //     return correct.calculate(indexerRed.get());
        // return correct.calculate(indexerBlue.get());
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

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Indexer");
        builder.addBooleanProperty("Top Occupied", this::topOccupied, null);
        builder.addBooleanProperty("Bottom Occupied", this::bottomOccupied, null);
        builder.addDoubleProperty("Indexer Top Percent Output", top::getMotorOutputPercent, null);
        builder.addDoubleProperty("Indexer Bottom Percent Output", bottom::getMotorOutputPercent, null);
        builder.addBooleanProperty("Wrong Color Ball in Intake", this::intakeHasWrongColor, null);
        // builder.addBooleanProperty("Red Input On", indexerRed::get, null);
        // builder.addBooleanProperty("Blue Input On", indexerBlue::get, null);
        builder.addDoubleProperty("Red Input PWM", indexerRed::getPeriod, null);
        builder.addDoubleProperty("Blue Input PWM", indexerBlue::getPeriod, null);
    }
}
