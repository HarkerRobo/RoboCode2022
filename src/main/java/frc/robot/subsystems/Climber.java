package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import harkerrobolib.wrappers.HSFalcon;

public class Climber extends SubsystemBase {
    private static Climber climber;

    private static final boolean MASTER_INVERTED = false;
    private static final boolean FOLLOWER_INVERTED = false;

    private static final double MAX_HEIGHT = 100000; //change
    private static final double MIDDLE_HEIGHT = 0; //change
    private static final double TOP_HEIGHT = 0; //change
    private static final double TRAVERSAL_HEIGHT = 0; //change

    private HSFalcon master;
    private HSFalcon follower;

    private Climber() {
        // master = new HSFalcon(RobotMap.CLIMBER_MASTER);
        // follower = new HSFalcon(Robotmap.CLIMBER_FOLLOWER);
    }

    public void init() {
        master.configFactoryDefault();
        follower.configFactoryDefault();

        follower.follow(master);

        master.setInverted(MASTER_INVERTED);
        follower.setInverted(FOLLOWER_INVERTED);
    }

    public void setClimberPosition(double pos) {
        if (Math.abs(pos) >= MAX_HEIGHT) {
            pos = MAX_HEIGHT * (pos / Math.abs(pos));
        }
        master.set(ControlMode.Position, pos);
    }

    public HSFalcon getClimberMaster() {
        return master;
    }

    public HSFalcon getClimberFollower() {
        return follower;
    }


    public static Climber getInstance() {
        if (climber == null) {
            climber = new Climber();
        }
        return climber;
    }
    
}
