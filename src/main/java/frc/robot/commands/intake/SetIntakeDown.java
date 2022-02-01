package frc.robot.commands.intake;


public class SetIntakeDown extends InstantCommand {
    public SetIntakeDown() {
        addRequirements(Intake.getInstance());
    }
    public void initialize() {
        Intake.getInstance().getSolenoid().set(Intake.DOWN);
    }
 }