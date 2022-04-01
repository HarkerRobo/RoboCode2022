package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import harkerrobolib.commands.IndefiniteCommand;

/**
 * Shoots with a set velocity in m/s
 */
public class ShootAgainstHub extends IndefiniteCommand {
    private static final double HUB_SPEED = 26.5;
    
    public ShootAgainstHub() {
        addRequirements(Shooter.getInstance());
    }
    
    public void execute() {
        Shooter.getInstance().setVelocity(HUB_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().setPercentOutput(0);
    }
}
