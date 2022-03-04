package frc.robot.commands.climber;

import frc.robot.OI;
import frc.robot.subsystems.Climber;
import harkerrobolib.commands.IndefiniteCommand;

public class ClimberManual extends IndefiniteCommand{
    public ClimberManual() {
        addRequirements(Climber.getInstance());
    }

    public void execute() {
        if (OI.getInstance().getDriverGamepad().getDownDPadButton().get()) 
            Climber.getInstance().setClimberOutput(-0.7);
        else if (OI.getInstance().getDriverGamepad().getUpDPadButton().get())
            Climber.getInstance().setClimberOutput(0.7);
        else 
            Climber.getInstance().setClimberOutput(-0.06);
    }
}
