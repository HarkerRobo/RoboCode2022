package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Stage1 extends SequentialCommandGroup{
    public Stage1() {
        super(new SetClimberPosition(0, 0.7));
    }
    
}
