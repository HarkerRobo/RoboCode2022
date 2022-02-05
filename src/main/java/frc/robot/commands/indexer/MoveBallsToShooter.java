package frc.robot.commands.indexer;

import frc.robot.subsystems.Indexer;
import harkerrobolib.commands.IndefiniteCommand;

public class MoveBallsToShooter extends IndefiniteCommand {
    private static final double OUTPUT = 0.4;
    public MoveBallsToShooter() {
        addRequirements(Indexer.getInstance());
    }

    public void execute() {
        Indexer.getInstance().setPercentOutputBoth(OUTPUT);
    }

    public void end(boolean interrupted) {
        Indexer.getInstance().setPercentOutputBoth(OUTPUT);
    }
}
