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
        referencePoints.put(1.18, 28.75);
        referencePoints.put(1.4, 29.5);
        referencePoints.put(1.58, 29.75);
        referencePoints.put(1.89, 30.5);
        referencePoints.put(2.27, 31.5);
        referencePoints.put(2.7, 33.0);
        referencePoints.put(2.88, 36.0);
        referencePoints.put(3.1, 38.0);
        referencePoints.put(4.49, 55.0);
    }
    
    public void execute() {
        if(Limelight.isTargetVisible()) velocity = referencePoints.get(Limelight.getDistance());
        else //if(OI.getInstance().getOperatorGamepad().getButtonBState()) 
            velocity = 32;
        SmartDashboard.putNumber("shooter ref point val", velocity);
        velocity = SmartDashboard.getNumber("desired velocity", 0);
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
