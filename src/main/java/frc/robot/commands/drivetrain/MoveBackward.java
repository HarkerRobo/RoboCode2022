// package frc.robot.commands.drivetrain;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import frc.robot.subsystems.Drivetrain;
// import harkerrobolib.commands.IndefiniteCommand;

// public class MoveBackward extends IndefiniteCommand{
//     public MoveBackward() {
//         addRequirements(Drivetrain.getInstance());
//     }

//     @Override
//     public void execute() {
//         ChassisSpeeds chassis = new ChassisSpeeds(1, 0, 0);
//         Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(chassis));
//     }

//     @Override
//     public void end(boolean isFinished){
//         ChassisSpeeds chassis = new ChassisSpeeds(0, 0, 0);
//         Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(chassis));
//     }
// }
