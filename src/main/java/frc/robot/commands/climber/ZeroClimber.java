package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ZeroClimber extends CommandBase{
    public ZeroClimber() {
        addRequirements(Climber.getInstance());
    }

    public void execute() {
        
    }
}
