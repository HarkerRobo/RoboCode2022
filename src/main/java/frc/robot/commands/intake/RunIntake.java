package frc.robot.commands.intake;

import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import harkerrobolib.commands.IndefiniteCommand;

public class RunIntake extends IndefiniteCommand{
    private static final double SPEED = 0.6;

    public RunIntake() {
        addRequirements(Intake.getInstance());
    }   

    public void execute() {
        Intake.getInstance().setVelocity(SPEED * Intake.MAX_RPS);
        Intake.getInstance().state = 1;
    }
    
    public void end(boolean interrupted) {
        Intake.getInstance().setPercentOutput(0);
        Intake.getInstance().state = 0;
    }
}
