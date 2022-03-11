package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

/**
 * Command to set the intake up
 */
public class SetIntakeUp extends InstantCommand {
    public SetIntakeUp() {
        addRequirements(Intake.getInstance());
    }
    public void initialize() {
        Intake.getInstance().getSolenoid().set(DoubleSolenoid.Value.kForward);
    }
 }