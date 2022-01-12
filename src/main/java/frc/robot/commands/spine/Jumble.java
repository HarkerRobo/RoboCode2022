package frc.robot.commands.spine;

import frc.robot.OI;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import harkerrobolib.commands.IndefiniteCommand;

public class Jumble extends IndefiniteCommand {
    private static final double AGITATOR_MAX_SPEED = 1;
    private static final double LINEAR_MAX_SPEED = 0.54;
    private static final double MAX_VELOCITY = 15;

    public Jumble(){
        addRequirements(Intake.getInstance());
        addRequirements(Indexer.getInstance());
    }

    @Override
    public void execute(){
        if(OI.getInstance().getDriverGamepad().getButtonYState()){
            Indexer.getInstance().setAgitatorPercentOutput(-AGITATOR_MAX_SPEED);
            Indexer.getInstance().setLinearPercentOutput(-LINEAR_MAX_SPEED);
            Intake.getInstance().setVelocity(MAX_VELOCITY);
        }
    }
}
