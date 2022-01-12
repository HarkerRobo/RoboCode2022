// package frc.robot.commands.spine;

// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Indexer;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Shooter;
// import harkerrobolib.commands.IndefiniteCommand;
// import harkerrobolib.util.MathUtil;

// import frc.robot.util.Vector;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.OI;

// public class LinearManual extends IndefiniteCommand {
//     private static final double AGITATOR_MAX_SPEED= 1;
//     private static final double LINEAR_MAX_SPEED= 0.54;


//     private long commandTime;
//     public LinearManual() {
//         addRequirements(Indexer.getInstance());
//     }

//     @Override
//     public void initialize() {
//         commandTime= System.currentTimeMillis();
//     }

//     @Override
//     public void execute(){
//         Indexer.getInstance().getSolenoid().set(Indexer.BLOCKER_CLOSED);

//         boolean blocked=Indexer.getInstance().shooterSensorBlocked() && Indexer.getInstance().linearSensorBlocked();
//         SmartDashboard.putBoolean("Blocked", blocked);
//         if(OI.getInstance().getDriverGamepad().getButtonXState()){

//             if(!blocked){
//             Indexer.getInstance().setLinearPercentOutput(LINEAR_MAX_SPEED);
//             }
//             if(commandTime%1000<500){
//                 Indexer.getInstance().setAgitatorPercentOutput(AGITATOR_MAX_SPEED);
//             }
//             else{
//                 Indexer.getInstance().setAgitatorPercentOutput(-AGITATOR_MAX_SPEED);
//             }
            
//             Indexer.getInstance().getSolenoid().set(Indexer.BLOCKER_CLOSED);
//         }
//         else{
//             Indexer.getInstance().setLinearPercentOutput(0);
//             Indexer.getInstance().setAgitatorPercentOutput(0);

//             Indexer.getInstance().getSolenoid().set(Indexer.BLOCKER_CLOSED);

//         }
//         commandTime= System.currentTimeMillis();

//     }
// }
