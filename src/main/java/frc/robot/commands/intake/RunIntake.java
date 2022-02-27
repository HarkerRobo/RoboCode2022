package frc.robot.commands.intake;

import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import harkerrobolib.commands.IndefiniteCommand;

public class RunIntake extends IndefiniteCommand{
    private static final double SPEED = 0.6;
    private double INDEX_SPEED = 0.35;

    public RunIntake() {
        addRequirements(Intake.getInstance());
        addRequirements(Indexer.getInstance());
    }   

    public void execute() {
        Intake.getInstance().setVelocity(SPEED * Intake.MAX_RPS); 
        if(!Indexer.getInstance().topOccupied())
            Indexer.getInstance().setPercentOutputTop(INDEX_SPEED);
        else
            Indexer.getInstance().setPercentOutputTop(0);
        if(Indexer.getInstance().topOccupied() && Indexer.getInstance().bottomOccupied())
            Indexer.getInstance().setPercentOutputBottom(0);
        else
            Indexer.getInstance().setPercentOutputBottom(INDEX_SPEED);
    }
    
    public void end(boolean interrupted) {
        Intake.getInstance().setPercentOutput(0);
    }
}
