package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import harkerrobolib.commands.IndefiniteCommand;

public class IntakeAutonControlReverse extends IndefiniteCommand {
    private Command runBackward;
    
    IntakeAutonControlReverse(double velocity) {    
        runBackward= new IntakeManualPID(velocity, true);    
        addRequirements(Intake.getInstance());
    }

    @Override
    public void initialize() {
        runBackward.initialize();
    }   

    @Override
    public void execute() {
        runBackward.execute();
    }

    @Override
    public void end(boolean a) {
        runBackward.end(true);
    }
}


/* void edu.wpi.first.wpilibj2.command.CommandBase.addRequirements(Subsystem... requirements)
Adds the specified requirements to the command.

Parameters:

requirements the requirements to add
The method addRequirements(Subsystem...) in the type CommandBase is not applicable for the arguments (Intake)Java(67108979)
*/