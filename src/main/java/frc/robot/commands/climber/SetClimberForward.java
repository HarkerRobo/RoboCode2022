package frc.robot.commands.climber;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotMap;
import frc.robot.subsystems.Climber;

public class SetClimberForward extends InstantCommand {
    public SetClimberForward() {
        addRequirements(Climber.getInstance());
    }

    public void initialize() {
        Climber.getInstance().getClimberPiston().set(RobotMap.IS_COMP ? Value.kReverse : Value.kForward);
    }
}