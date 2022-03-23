package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter;
import frc.robot.util.InterpolatedTreeMap;
import frc.robot.util.Limelight;
import harkerrobolib.commands.IndefiniteCommand;

/**
 * Shoots with limelight
 */
public class ShootWithLimelight extends IndefiniteCommand {
    private InterpolatedTreeMap referencePoints;

    private double velocity;
    
    public ShootWithLimelight() {
        addRequirements(Shooter.getInstance());
        referencePoints = new InterpolatedTreeMap();
        referencePoints.put(1.36, 26.5);
        referencePoints.put(1.54, 29.0);
        referencePoints.put(1.75, 28.5);
        referencePoints.put(1.91, 29.0);
        referencePoints.put(2.17, 28.5);
        referencePoints.put(2.4, 29.5);
        referencePoints.put(2.62, 31.0);
        referencePoints.put(2.84, 32.0);
        referencePoints.put(3.11, 34.5);
        referencePoints.put(3.43, 37.5);
        referencePoints.put(3.73, 41.0);
        referencePoints.put(4.25, 61.0);
        referencePoints.put(4.52, 64.0);
    }
    
    public void execute() {
        velocity = referencePoints.get(Limelight.getDistance());
        // velocity = SmartDashboard.getNumber("desired velocity", 0);
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
