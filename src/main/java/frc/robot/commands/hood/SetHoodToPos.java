package frc.robot.commands.hood;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Units;
import frc.robot.subsystems.Hood;
import harkerrobolib.commands.IndefiniteCommand;

public class SetHoodToPos extends IndefiniteCommand{
    private double hoodPos;

    public SetHoodToPos(double pos) {
        addRequirements(Hood.getInstance());
        hoodPos = pos;
    }

    public void initialize() {
        Hood.getInstance().getController().reset(Hood.getInstance().getHoodPos());
    }
    
    public void execute() {
        if(!Hood.isZeroed) return;

        double controlEffort = Hood.getInstance().getController().calculate(Hood.getInstance().getHoodPos(), hoodPos);
        double feedforwardAmount = Hood.getInstance().getFeedForward().calculate(Hood.getInstance().getController().getSetpoint().velocity);
        Hood.getInstance().getHood().set(ControlMode.PercentOutput, (controlEffort + feedforwardAmount)/Units.MAX_CONTROL_EFFORT);
    }

    public void end(boolean interrupted) {
        Hood.getInstance().getHood().set(ControlMode.PercentOutput, 0);
    }
}
