package frc.robot.commands.climber;

import frc.robot.OI;
import frc.robot.subsystems.Climber;
import harkerrobolib.commands.IndefiniteCommand;

public class ClimberManual extends IndefiniteCommand{

    public static final double MAGNITUDE_UP = 0.5;
    public static final double MAGNITUDE_BACKWARD = -0.5;
    public static final double MAGNITUDE_IDLE = -0.05;

    public ClimberManual() {
        addRequirements(Climber.getInstance());
    }

    public void execute() {
        if (OI.getInstance().getDriverGamepad().getDownDPadButton().get()){
            Climber.getInstance().setClimberOutputLeft(MAGNITUDE_BACKWARD);
            Climber.getInstance().setClimberOutputRight(MAGNITUDE_BACKWARD);
        }
        else if (OI.getInstance().getDriverGamepad().getUpDPadButton().get()){
            Climber.getInstance().setClimberOutputLeft(MAGNITUDE_UP);
            Climber.getInstance().setClimberOutputRight(MAGNITUDE_UP);
        }
        else {
            Climber.getInstance().setClimberOutputLeft(MAGNITUDE_IDLE);
            Climber.getInstance().setClimberOutputRight(MAGNITUDE_IDLE);
        }
    }
}
