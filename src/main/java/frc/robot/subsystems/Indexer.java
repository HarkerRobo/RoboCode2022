package frc.robot.subsystems;

import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import harkerrobolib.wrappers.HSFalcon;

public class Indexer extends SubsystemBase {

    private static Indexer indexer;

    private HSFalcon top;
    private HSFalcon bottom;

    private static final boolean TOP_INVERT = false;
    private static final boolean BOTTOM_INVERT = false;

    private Indexer() {
        // top = new HSFalcon(RobotMap.INDEXER_TOP);
        // bottom = new HSFalcon(RobotMap.INDEXER_BOTTOM);
    }

    private void init() {
        top.configFactoryDefault();
        top.setInverted(TOP_INVERT);
        top.configVelocityMeasurementWindow(1);
        top.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
    }

    public static Indexer getInstance() {
        if(indexer == null)
            indexer = new Indexer();
        return indexer;
    }
}
