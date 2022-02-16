package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter;
import harkerrobolib.commands.IndefiniteCommand;

/**
 * Shoots with a set velocity in m/s
 */
public class ShootWithVelocity extends IndefiniteCommand {
    private double vel;
    
    public ShootWithVelocity(double velocity) {
        vel = velocity;
        addRequirements(Shooter.getInstance());
    }
    
    public void execute() {
        vel = SmartDashboard.getNumber("desired velocity", 0);
        Shooter.getInstance().setVelocity(vel);
        SmartDashboard.putNumber("current vel", Shooter.getInstance().getWheelRPS());
        SmartDashboard.putNumber("kalman output", Shooter.getInstance().getVelocitySystem().getVelocity());
        SmartDashboard.putNumber("current output", Shooter.getInstance().getVelocitySystem().getOutput());
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().setPercentOutput(0);
    }
}
