package frc.robot.commands.shooter;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.Shooter;
import harkerrobolib.commands.IndefiniteCommand;

public class ShootWithVelocity extends IndefiniteCommand {
    private double vel;
    
    public ShootWithVelocity(double velocity) {
        vel = velocity;
        addRequirements(Shooter.getInstance());
    }
    
    public void execute() {
        vel = SmartDashboard.getNumber("desired velocity", 0);
        Shooter.getInstance().getVelocitySystem().set(vel);
        Shooter.getInstance().getVelocitySystem().update(Shooter.getInstance().getRawVelocity());
        Shooter.getInstance().setPercentOutput(Shooter.getInstance().getVelocitySystem().getOutput());
        SmartDashboard.putNumber("current vel", Shooter.getInstance().getRawVelocity());
        SmartDashboard.putNumber("kalman output", Shooter.getInstance().getVelocitySystem().getVelocity());
        SmartDashboard.putNumber("current output", Shooter.getInstance().getVelocitySystem().getOutput());
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().setPercentOutput(0);
    }
}
