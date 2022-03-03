package frc.robot.commands.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class SetClimberPosition extends CommandBase {
    private static final double OUTPUT_MAGNITUDE = 1;
    private double MAX_ERROR = 1000;
    private double position;

    public SetClimberPosition(double pos) {
        addRequirements(Climber.getInstance());
        this.position = pos;
    }

    public void execute() {
        double direction = Math.signum(position - Climber.getInstance().getPosition());
        Climber.getInstance().setClimberOutput(direction * OUTPUT_MAGNITUDE);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(position - Climber.getInstance().getPosition()) < MAX_ERROR;
    }

    public void end(boolean interrupted) {
        Climber.getInstance().getClimberMaster().set(ControlMode.PercentOutput, 0);
    }
}