package frc.robot.commands.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class SetClimberPosition extends CommandBase {
    private static final double OUTPUT_MAGNITUDE_FORWARD = 0.5;
    private static final double OUTPUT_MAGNITUDE_BACKWARD = -0.7;
    private double MAX_ERROR = 500;
    private double position;
    private double speed;

    public SetClimberPosition(double pos, double speed) {
        addRequirements(Climber.getInstance());
        this.position = pos;
        this.speed = speed;
    }

    public void execute() {
        if(!Climber.getInstance().isZeroed) return;
        double direction = Math.signum(position - Climber.getInstance().getPosition());
        Climber.getInstance().setClimberOutput(direction * Math.abs(speed));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(position - Climber.getInstance().getPosition()) < MAX_ERROR;
    }

    public void end(boolean interrupted) {
        Climber.getInstance().getClimberMaster().set(ControlMode.PercentOutput, 0);
    }
}