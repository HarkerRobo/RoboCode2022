// package frc.robot.commands.shooter;

// import frc.robot.subsystems.Shooter;
// import frc.robot.util.InterpolatedTreeMap;
// import frc.robot.util.Limelight;
// import harkerrobolib.commands.IndefiniteCommand;

// /**
//  * Shoots with a set velocity in m/s
//  */
// public class ShooterManual extends IndefiniteCommand {
//     private InterpolatedTreeMap referencePoints;

//     private double velocity;
    
//     public ShooterManual() {
//         addRequirements(Shooter.getInstance());
//         referencePoints = new InterpolatedTreeMap();
//         // if(RobotMap.IS_COMP) { 
//             referencePoints.put(0.94, 27.0);
//             referencePoints.put(1.15, 27.0);
//             referencePoints.put(1.3, 27.5);
//             referencePoints.put(1.54, 28.0);
//             referencePoints.put(1.75, 29.0);
//             referencePoints.put(1.9, 30.5);
//             referencePoints.put(2.26, 34.0);
//             referencePoints.put(2.5, 34.5);
//             referencePoints.put(2.72, 35.0);
//             referencePoints.put(2.99, 38.5);
//             referencePoints.put(3.18, 45.5);
//             referencePoints.put(3.39, 51.5);
//             referencePoints.put(4.0, 56.5);
//         // } else {
//         //     referencePoints.put(1.08, 28.0);
//         //     referencePoints.put(1.18, 29.0);
//         //     referencePoints.put(1.33, 27.0); //prac
//         //     // referencePoints.put(1.4, 29.75);
//         //     referencePoints.put(1.58, 29.5);
//         //     referencePoints.put(1.89, 30.5);
//         //     referencePoints.put(2.27, 31.5);
//         //     referencePoints.put(2.7, 33.0);
//         //     referencePoints.put(2.88, 36.0);
//         //     referencePoints.put(3.1, 38.0);
//         //     referencePoints.put(3.67, 55.0);
//         //     referencePoints.put(4.29, 63.0);
//         // }
    
//     }
    
//     public void execute() {
//         if(Limelight.isTargetVisible()) velocity = referencePoints.get(Limelight.getDistance());
//         else //if(OI.getInstance().getOperatorGamepad().getButtonBState()) 
//             velocity = 32;
//         Shooter.getInstance().setVelocity(velocity);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         velocity = 0;
//         Shooter.getInstance().setPercentOutput(velocity);
//     }
// }
