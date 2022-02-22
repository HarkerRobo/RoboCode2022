package frc.robot.commands.indexer;

import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import harkerrobolib.commands.IndefiniteCommand;

public class IndexerManual extends IndefiniteCommand {
    private double SPEED = 0.3;
    
    public IndexerManual() {
        addRequirements(Indexer.getInstance());
    }

    public void execute() {
        if(Intake.getInstance().state == 1){
            if(!Indexer.getInstance().topOccupied())
                Indexer.getInstance().setPercentOutputTop(SPEED);
            else
                Indexer.getInstance().setPercentOutputTop(0);
            if(Indexer.getInstance().topOccupied() && Indexer.getInstance().bottomOccupied())
                Indexer.getInstance().setPercentOutputBottom(0);
            else
                Indexer.getInstance().setPercentOutputBottom(SPEED);
        } else if(Intake.getInstance().state == -1){
            Indexer.getInstance().setPercentOutputTop(-SPEED);
            Indexer.getInstance().setPercentOutputBottom(-SPEED);
        }
        else {
            if(!Indexer.getInstance().topOccupied() && Indexer.getInstance().bottomOccupied()) {
                Indexer.getInstance().setPercentOutputBottom(SPEED);
                Indexer.getInstance().setPercentOutputTop(SPEED);
            }
            else {
                Indexer.getInstance().setPercentOutputBottom(0);
                Indexer.getInstance().setPercentOutputTop(0);
            }
        }
    }
}
