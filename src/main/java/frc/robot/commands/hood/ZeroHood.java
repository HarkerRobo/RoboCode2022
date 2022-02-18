package frc.robot.commands.hood;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class ZeroHood extends CommandBase {
    public ZeroHood(){
        addRequirements(Hood.getInstance());
    }

    public void execute(){
        Hood.getInstance().getHood().set(ControlMode.PercentOutput, -0.08);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(Hood.getInstance().getHood().getStatorCurrent()) >= Shooter.HOOD_STALLING_CURRENT;
    }

    public void end(boolean interrupted){
        Hood.getInstance().getHood().set(ControlMode.PercentOutput, 0);
        Hood.getInstance().setHoodOffset();
        Hood.getInstance().getHood().setSelectedSensorPosition(0);
        Hood.isZeroed = true;
    }
}
