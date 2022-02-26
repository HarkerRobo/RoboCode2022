package frc.robot.commands.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class SetClimberPosition extends CommandBase {
    private double MAX_ERROR = 100;
    private double position;

    public SetClimberPosition(double pos) {
        addRequirements(Climber.getInstance());
        this.position = pos;
    }

    public void execute() {
        Climber.getInstance().setClimberPosition(position);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Climber.getInstance().getClimberMaster().getClosedLoopError()) < MAX_ERROR;
    }

    public void end(boolean interrupted) {
        Climber.getInstance().getClimberMaster().set(ControlMode.PercentOutput, 0);
    }
}