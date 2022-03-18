package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.subsystems.Shooter;
import frc.robot.util.InterpolatedTreeMap;
import frc.robot.util.Limelight;
import harkerrobolib.commands.IndefiniteCommand;

/**
 * Shoots with a set velocity in m/s
 */
public class ShooterManual extends IndefiniteCommand {
    private InterpolatedTreeMap referencePoints;

    private double velocity;
    
    public ShooterManual() {
        addRequirements(Shooter.getInstance());
        referencePoints = new InterpolatedTreeMap();
        referencePoints.put(0.94, 27.0);
        referencePoints.put(1.15, 27.0);
        referencePoints.put(1.3, 27.5);
        referencePoints.put(1.54, 28.0);
        referencePoints.put(1.75, 29.0);
        referencePoints.put(1.9, 30.5);
        referencePoints.put(2.26, 33.5);
        referencePoints.put(2.5, 33.5);
        referencePoints.put(2.72, 34.5);
        referencePoints.put(2.99, 38.0);
        referencePoints.put(3.18, 45.0);
        referencePoints.put(3.39, 51.0);
        referencePoints.put(4.0, 56.0);
    }
    
    public void execute() {
        if(Limelight.isTargetVisible()) velocity = referencePoints.get(Limelight.getDistance());
        else //if(OI.getInstance().getOperatorGamepad().getButtonBState()) 
            velocity = 32;
        // SmartDashboard.putNumber("shooter ref point val", velocity);
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
        Shooter.getInstance().setPercentOutput(velocity);
    }
}
