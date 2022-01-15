package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import harkerrobolib.wrappers.HSFalcon;

public class Shooter extends SubsystemBase {
    private static Shooter shooter;
    private HSFalcon master;
    private HSFalcon follower;
    
    private Shooter(){
        master = new HSFalcon(RobotMap.SHOOTER_MASTER);
        follower = new HSFalcon(RobotMap.SHOOTER_FOLLOWER);
        initMotors();
    }

    public void initMotors() {
        follower.follow(master);
    }

    public void setVelocity(double vel) {
        master.set(ControlMode.Velocity, vel);
    }
    
    public static Shooter getInstance() {
        if (shooter == null)
        {
            shooter = new Shooter();
        }
        return shooter;
    }
}