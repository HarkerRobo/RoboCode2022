package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import harkerrobolib.wrappers.HSFalcon;

public class Indexer extends SubsystemBase {

    private static Indexer indexer;

    private HSFalcon top;
    private HSFalcon bottom;

    private static final boolean TOP_INVERT = true;
    private static final boolean BOTTOM_INVERT = false;

    // private ColorSensorV3 colorSensor;
    // private DigitalInput topProximity;

    private Indexer() {
        top = new HSFalcon(RobotMap.INDEXER_TOP);
        bottom = new HSFalcon(RobotMap.INDEXER_BOTTOM);
        // colorSensor = new ColorSensorV3(Port.kOnboard);
        init();
    }

    private void init() {
        top.configFactoryDefault();
        top.setInverted(TOP_INVERT);
        bottom.configFactoryDefault();
        bottom.setInverted(BOTTOM_INVERT);
    }

    public boolean bottomOccupied() {
        return false;//colorSensor.getProximity() > 1024;
    }

    public boolean topOccupied() {
        return false;//!topProximity.get();
    }

    // public Color getColor() {
    //     return colorSensor.getColor();
    // }

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
