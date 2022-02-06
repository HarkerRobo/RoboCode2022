package frc.robot.commands.indexer;

import frc.robot.subsystems.Indexer;
import harkerrobolib.commands.IndefiniteCommand;

public class MoveBallsToShooter extends IndefiniteCommand {
    
    public static final double SPEED = 0.4;
    
    public MoveBallsToShooter() {
        addRequirements(Indexer.getInstance());
    }
    
    public void execute() {
        Indexer.getInstance().setPercentOutputBottom(SPEED);
        Indexer.getInstance().setPercentOutputTop(SPEED);
    }
}