package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.Autons;
import frc.robot.commands.drivetrain.AlignWithLimelight;
import frc.robot.commands.drivetrain.SwerveManual;
import frc.robot.commands.hood.ZeroHood;
import frc.robot.commands.indexer.MoveBallsToShooter;
import frc.robot.commands.intake.SetIntakeUp;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.shooter.ShootWithVelocity;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.wrappers.HSGamepad;
import harkerrobolib.wrappers.XboxGamepad;

/**
 * Defines drive and operator gamepad and binds definite commands
 */
public class OI {
    private static OI oi;

    private HSGamepad driverGamepad;
    private HSGamepad operatorGamepad;

    public static final double DEADBAND = 0.15;

    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    
    private OI() {
        driverGamepad = new XboxGamepad(DRIVER_PORT);
        operatorGamepad = new XboxGamepad(OPERATOR_PORT);
        initBindings();
    }

    public void initBindings() {
        operatorGamepad.getButtonBumperLeft().whilePressed(new ShootWithVelocity());
        operatorGamepad.getButtonBumperRight().whilePressed(new MoveBallsToShooter());
        operatorGamepad.getButtonY().whenPressed(new ToggleIntake());
        driverGamepad.getButtonY().whenPressed(new ToggleIntake());
        driverGamepad.getButtonX().whenPressed(new InstantCommand(() -> {
            Drivetrain.getInstance().getPigeon().setYaw(0); 
            SwerveManual.pigeonAngle = 0;
        }));
        driverGamepad.getButtonB().whilePressed(new AlignWithLimelight());
        driverGamepad.getButtonA().whenPressed(new ZeroHood());
    }

    public HSGamepad getDriverGamepad(){
        return driverGamepad;
    }
    public HSGamepad getOperatorGamepad(){
        return operatorGamepad;
    }

    public static OI getInstance(){
        if(oi == null)
            oi = new OI();
        return oi;
    }
}
