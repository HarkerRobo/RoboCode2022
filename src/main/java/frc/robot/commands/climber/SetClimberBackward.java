package frc.robot.commands.climber;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Climber;

public class SetClimberBackward extends InstantCommand {
    public SetClimberBackward() {
        // addRequirements(Climber.getInstance());
    }

    public void initialize() {
        Climber.getInstance().getClimberPiston().set(Value.kReverse);
    }
}