package frc.robot.commands.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class MaxHood extends CommandBase {
    public MaxHood(){
        addRequirements(Shooter.getInstance());
    }

    public void execute(){
        Shooter.getInstance().getHood().set(ControlMode.PercentOutput, 0.08);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(Shooter.getInstance().getHood().getStatorCurrent()) >= Shooter.HOOD_STALLING_CURRENT;
    }

    public void end(boolean interrupted){
        Shooter.getInstance().getHood().set(ControlMode.PercentOutput, 0);
        Shooter.getInstance().setHoodOffset();
    }
}
