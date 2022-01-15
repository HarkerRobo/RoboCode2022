package frc.robot.commands.intake;

import harkerrobolib.commands.IndefiniteCommand;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.Intake;
import frc.robot.OI;
import frc.robot.RobotMap;

public class IntakeManual extends IndefiniteCommand{
    public static final double VELOCITY = 10;
    public IntakeManual()
    {
        addRequirements(Intake.getInstance());
    }
    public void execute()
    {
        if (OI.getInstance().getDriverGamepad().getButtonAState()){
            Intake.getInstance().setVelocity(VELOCITY);
        }
    }
}