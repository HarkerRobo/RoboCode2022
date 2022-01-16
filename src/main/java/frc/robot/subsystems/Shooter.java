package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import harkerrobolib.wrappers.HSFalcon;

public class Shooter extends SubsystemBase {
    private static Shooter shooter;


    private static final boolean MASTER_INVERTED = false;
    private static final boolean FOLLOWER_INVERTED = false;

    private HSFalcon master;
    private HSFalcon follower;
    
    private Shooter(){
        master = new HSFalcon(RobotMap.SHOOTER_MASTER);
        follower = new HSFalcon(RobotMap.SHOOTER_FOLLOWER);
        initMotors();
    }

    public void initMotors() {
        follower.follow(master);

        master.setInverted(MASTER_INVERTED);
        follower.setInverted(FOLLOWER_INVERTED);
    }

    public void setVelocity(double vel) {
        master.set(ControlMode.Velocity, vel);
    }
    public void setPercentOutput(double speed) {
        master.set(ControlMode.PercentOutput, speed);
    }
    public static Shooter getInstance() {
        if (shooter == null)
        {
            shooter = new Shooter();
        }
        return shooter;
    }
}