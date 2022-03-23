package frc.robot.commands.indexer;

import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import harkerrobolib.commands.IndefiniteCommand;

public class IndexerManual extends IndefiniteCommand {
    private static final double INTAKE_SPEED = 0.35;
    private static final double INTAKE_BOTTOM_SPEED = 0.9;
    private double OUTTAKE_SPEED = -0.7;
    
    public IndexerManual() {
        addRequirements(Indexer.getInstance());
    }

    public void execute() {
        if(Intake.getInstance().state == 1){
            if(!Indexer.getInstance().topOccupied()) {
                Indexer.getInstance().setPercentOutputTop(INTAKE_SPEED);
            }
            else {
                Indexer.getInstance().setPercentOutputTop(0);
            }
            
            if(Indexer.getInstance().topOccupied() && Indexer.getInstance().bottomOccupied()) 
                Indexer.getInstance().setPercentOutputBottom(0);    
            else if(Indexer.getInstance().intakeHasWrongColor())
                Indexer.getInstance().setPercentOutputBottom(-INTAKE_SPEED);
            else
                Indexer.getInstance().setPercentOutputBottom(INTAKE_BOTTOM_SPEED);
            
        } 
        else if(Intake.getInstance().state == -1){
            Indexer.getInstance().setPercentOutputTop(OUTTAKE_SPEED);
            Indexer.getInstance().setPercentOutputBottom(OUTTAKE_SPEED);
        }
        else {
            // if(!Indexer.getInstance().topOccupied() && Indexer.getInstance().bottomOccupied()) {
            //     Indexer.getInstance().setPercentOutputBottom(SPEED);
            //     Indexer.getInstance().setPercentOutputTop(SPEED);
            // }
            // else {
                Indexer.getInstance().setPercentOutputBottom(0);
                Indexer.getInstance().setPercentOutputTop(0);
            // }
        }
    }

    @Override
    public void end(boolean interrupted) {
        Indexer.getInstance().setPercentOutputBottom(0);
        Indexer.getInstance().setPercentOutputTop(0);
    }
}
