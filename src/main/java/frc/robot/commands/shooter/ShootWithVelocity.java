package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.util.InterpolatedTreeMap;
import frc.robot.util.Limelight;
import harkerrobolib.commands.IndefiniteCommand;

/**
 * Shoots with a set velocity in m/s
 */
public class ShootWithVelocity extends IndefiniteCommand {
    private InterpolatedTreeMap referencePoints;

    private double velocity;
    
    public ShootWithVelocity() {
        addRequirements(Shooter.getInstance());

        referencePoints.put(0.17, 27.0);
        referencePoints.put(0.7, 30.0);
        referencePoints.put(1.0, 31.0);
        referencePoints.put(1.3, 31.0);
        referencePoints.put(1.75, 28.0);
        referencePoints.put(1.98, 29.0);
        referencePoints.put(2.38, 31.0);
        referencePoints.put(2.8, 33.0);
        referencePoints.put(3.24, 33.0);
        referencePoints.put(3.7, 39.0);
        referencePoints.put(4.1, 43.0);
    }
    
    public void execute() {
        if(Limelight.isTargetVisible()) {
            velocity = referencePoints.get(Limelight.getDistance());
        }

        Shooter.getInstance().setVelocity(velocity);
        SmartDashboard.putNumber("current vel", Shooter.getInstance().getWheelRPS());
        SmartDashboard.putNumber("kalman output", Shooter.getInstance().getVelocitySystem().getVelocity());
        SmartDashboard.putNumber("current output", Shooter.getInstance().getVelocitySystem().getOutput());
        // SmartDashboard.putNumber("shooter control effort", Shooter.getInstance().getMaster().getMotorOutputVoltage()/10);
    }

    @Override
    public void end(boolean interrupted) {
        velocity = 0;
        Shooter.getInstance().setPercentOutput(0);
    }
}
