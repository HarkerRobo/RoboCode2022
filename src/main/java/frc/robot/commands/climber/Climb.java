// package frc.robot.commands.climber;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import frc.robot.OI;
// import frc.robot.subsystems.Climber;

// public class Climb extends SequentialCommandGroup{
//     public Climb() {
//         addCommands(new SetClimberPosition(Climber.MAX_HEIGHT));
//         addCommands(new WaitUntilCommand(()->OI.getInstance().getDriverGamepad().getButtonStart().get()));
//         addCommands(new SetClimberPosition(Climber.ZERO_HEIGHT));
//         addCommands(new WaitUntilCommand(()->OI.getInstance().getDriverGamepad().getButtonStart().get()));
//         addCommands(new SetClimberPosition(Climber.DOWN_HEIGHT));
//         addCommands(new ToggleClimber());
//         addCommands(new SetClimberPosition(Climber.MAX_HEIGHT));
//         addCommands(new ToggleClimber());
//         addCommands(new SetClimberPosition(Climber.ZERO_HEIGHT));
//         addCommands(new WaitUntilCommand(()->OI.getInstance().getDriverGamepad().getButtonStart().get()));
//         addCommands(new SetClimberPosition(Climber.DOWN_HEIGHT));
//         addCommands(new ToggleClimber());
//         addCommands(new SetClimberPosition(Climber.MAX_HEIGHT));
//         addCommands(new ToggleClimber());
//         addCommands(new SetClimberPosition(Climber.ZERO_HEIGHT));
//     }
// }
