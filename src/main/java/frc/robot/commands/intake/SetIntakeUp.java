package frc.robot.commands.intake;

public class SetIntakeUp extends InstantCommand {
    public SetIntakeUp() {
        addRequirements(Intake.getInstance());
    }
    public void initialize() {
        Intake.getInstance().getSolenoid().set(Intake.UP);
    }
 }