package frc.robot.commands.indexer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import harkerrobolib.commands.IndefiniteCommand;

public class MoveBallsToShooter extends IndefiniteCommand {
    
    public static final double SPEED = 0.3;
    public static final double LIMELIGHT_THRESHOLD = 10;
    private static Debouncer debouncer = new Debouncer(0.07, DebounceType.kFalling);
    
    public MoveBallsToShooter() {
        addRequirements(Indexer.getInstance());
    }
    
    public void execute() {
        if(debouncer.calculate(Math.abs(Shooter.getInstance().getWheelRPS() - 
            Shooter.getInstance().getVelocitySystem().getLinearSystemLoop().getNextR(0)) <= 3)) {
            Indexer.getInstance().setPercentOutputBottom(SPEED);
            Indexer.getInstance().setPercentOutputTop(SPEED);
        }
        else {
            Indexer.getInstance().setPercentOutputBottom(0);
            Indexer.getInstance().setPercentOutputTop(0);
        }
    }
}