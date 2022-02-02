package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class SetIntakeUp extends InstantCommand {
    public SetIntakeUp() {
        addRequirements(Intake.getInstance());
    }
    public void initialize() {
        Intake.getInstance().getSolenoid().set(Intake.UP);
    }
 }