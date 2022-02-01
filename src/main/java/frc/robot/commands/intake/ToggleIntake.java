package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

/**
 * Command to toggle the intake
 */
public class ToggleIntake extends InstantCommand {
    public ToggleIntake() {
        addRequirements(Intake.getInstance());
    }

    public void initialize() {
        Intake.getInstance().toggle();
    }
}