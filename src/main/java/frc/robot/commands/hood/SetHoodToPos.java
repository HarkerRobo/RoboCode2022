package frc.robot.commands.hood;

import frc.robot.subsystems.Hood;
import harkerrobolib.commands.IndefiniteCommand;

public class SetHoodToPos extends IndefiniteCommand{
    private double hoodPos;
    public SetHoodToPos(double pos) {
        addRequirements(Hood.getInstance());
        hoodPos = pos;
    }

    public void execute() {
        
    }
}
