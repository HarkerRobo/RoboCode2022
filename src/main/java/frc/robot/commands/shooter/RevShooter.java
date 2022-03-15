package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import harkerrobolib.commands.IndefiniteCommand;

public class RevShooter extends IndefiniteCommand{
    public RevShooter(){
        addRequirements(Shooter.getInstance());
    }

    public void execute(){
        Shooter.getInstance().setVelocity(40);
    }

    public void end(boolean isFinished){
        // Shooter.getInstance().setVelocity(0);
    }
}
