package frc.robot.commands.indexer;

import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Limelight;
import harkerrobolib.commands.IndefiniteCommand;

public class MoveBallsToShooter extends IndefiniteCommand {
    
    public static final double SPEED = 0.7;
    public static final double LIMELIGHT_THRESHOLD = 7;
    
    public MoveBallsToShooter() {
        addRequirements(Indexer.getInstance());
    }
    
    public void execute() {
        if(Math.abs(Shooter.getInstance().getWheelRPS() - 
            Shooter.getInstance().getVelocitySystem().getLinearSystemLoop().getNextR(0)) <= 3 && 
                (!Limelight.isTargetVisible() || Math.abs(Limelight.getTx()) < LIMELIGHT_THRESHOLD)) {
            Indexer.getInstance().setPercentOutputBottom(SPEED);
            Indexer.getInstance().setPercentOutputTop(SPEED);
        }
        else {
            Indexer.getInstance().setPercentOutputBottom(0);
            Indexer.getInstance().setPercentOutputTop(0);
        }
    }
}