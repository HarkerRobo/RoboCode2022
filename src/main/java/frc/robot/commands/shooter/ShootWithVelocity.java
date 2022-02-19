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
    private double vel;
    
    public ShootWithVelocity() {
        vel = Shooter.referencePoints.get((Limelight.isTargetVisible()) ? Limelight.getDistance() : 0.17);
        addRequirements(Shooter.getInstance());
    }
    
    public void execute() {
        Shooter.getInstance().setVelocity(vel);
        SmartDashboard.putNumber("current vel", Shooter.getInstance().getWheelRPS());
        SmartDashboard.putNumber("kalman output", Shooter.getInstance().getVelocitySystem().getVelocity());
        SmartDashboard.putNumber("current output", Shooter.getInstance().getVelocitySystem().getOutput());
        // SmartDashboard.putNumber("shooter control effort", Shooter.getInstance().getMaster().getMotorOutputVoltage()/10);
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().setPercentOutput(0);
    }
}
