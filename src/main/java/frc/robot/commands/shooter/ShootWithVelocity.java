package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import harkerrobolib.commands.IndefiniteCommand;

public class ShootWithVelocity extends IndefiniteCommand {
    private double vel;
    
    public ShootWithVelocity(double velocity) {
        vel = velocity;

    }
    public void execute() {
        Shooter.getInstance().setVelocity(vel);
        
    }
    public void end() {
        Shooter.getInstance().setVelocity(0);
        
    }
}