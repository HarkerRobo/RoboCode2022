package frc.robot.commands.hood;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class MaxHood extends CommandBase {
    public MaxHood(){
        addRequirements(Hood.getInstance());
    }

    public void execute(){
        Hood.getInstance().getHood().set(ControlMode.PercentOutput, 0.1);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(Hood.getInstance().getHood().getStatorCurrent()) >= Shooter.HOOD_STALLING_CURRENT;
    }

    public void end(boolean interrupted){
        Hood.getInstance().getHood().set(ControlMode.PercentOutput, 0);
        Hood.getInstance().setHoodMax();
    }
}
