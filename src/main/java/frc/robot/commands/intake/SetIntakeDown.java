package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

/**
 * Sets the intake down
 */
public class SetIntakeDown extends InstantCommand {
    public SetIntakeDown() {
        addRequirements(Intake.getInstance());
    }
    public void initialize() {
        Intake.getInstance().getSolenoid().set(Intake.DOWN);
    }
 }