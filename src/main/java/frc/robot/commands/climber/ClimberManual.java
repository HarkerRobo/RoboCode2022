package frc.robot.commands.climber;

import frc.robot.OI;
import frc.robot.subsystems.Climber;
import harkerrobolib.commands.IndefiniteCommand;

public class ClimberManual extends IndefiniteCommand{

    public static final double MAGNITUDE_UP = 0.5;
    public static final double MAGNITUDE_BACKWARD = -0.7;
    public static final double MAGNITUDE_IDLE = -0.06;

    public ClimberManual() {
        
        addRequirements(Climber.getInstance());
    }

    public void execute() {
        if (OI.getInstance().getOperatorGamepad().getDownDPadButton().get()){
            System.out.println("alex");
            Climber.getInstance().setClimberOutput(MAGNITUDE_BACKWARD);
        }
        else if (OI.getInstance().getOperatorGamepad().getUpDPadButton().get()){
            Climber.getInstance().setClimberOutput(MAGNITUDE_UP);
        }
        else {
            if(Climber.getInstance().getPositionLeft() > Climber.STOP_GOING_DOWN_HEIGHT) {
                Climber.getInstance().setClimberOutput(0);
            } else {
                Climber.getInstance().setClimberOutput(MAGNITUDE_IDLE);
            }
        }
    }
}
