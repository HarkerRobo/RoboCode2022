package frc.robot.commands.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class SetClimberPosition extends CommandBase {
    private static final double OUTPUT_MAGNITUDE_FORWARD = 0.5;
    private static final double OUTPUT_MAGNITUDE_BACKWARD = -0.7;
    private static final double MAX_ERROR = 500;
    private boolean limitSwitch;
    private double position;
    private double speed;

    public SetClimberPosition(double pos, double speed, boolean limitSwitch) {
        addRequirements(Climber.getInstance());
        this.position = pos;
        this.speed = speed;
        this.limitSwitch = limitSwitch;
    }

    public void execute() {
        if(!Climber.getInstance().isZeroed) return;
        double direction = Math.signum(position - Climber.getInstance().getPosition());
        Climber.getInstance().setClimberOutput(direction * Math.abs(speed));
    }

    @Override
    public boolean isFinished() {
        if(limitSwitch)
            return Climber.getInstance().getClimberMaster().isRevLimitSwitchClosed() == 1;
        return Math.abs(position - Climber.getInstance().getPosition()) < MAX_ERROR;
    }

    public void end(boolean interrupted) {
        Climber.getInstance().getClimberMaster().set(ControlMode.PercentOutput, 0);
    }
}