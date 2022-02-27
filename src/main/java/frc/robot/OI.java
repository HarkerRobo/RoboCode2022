package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.Autons;
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
        // driverGamepad.getButtonB().whenPressed(new InstantCommand(() -> {
        //     Intake.getInstance().toggle();
        // }, Intake.getInstance()));
        // driverGamepad.getButtonX().whilePressed(new ShootWithVelocity(20));
        // driverGamepad.getButtonY().whilePressed(new ShootWithVelocity(10));
        // driverGamepad.getButtonA().whilePressed(new ParallelCommandGroup(new ShootWithVelocity(), new MoveBallsToShooter()));
        // driverGamepad.getButtonA().whilePressed(new InstantCommand(Drivetrain.getInstance()::toggleFieldCentric));
        // driverGamepad.getButtonB().whilePressed(new IntakeManual(0.3));
        driverGamepad.getButtonY().whenPressed(new ToggleIntake());
        // driverGamepad.getButtonA().whilePressed(new SetIntakeDown());
        driverGamepad.getButtonX().whenPressed(new InstantCommand(() -> {
            Drivetrain.getInstance().getPigeon().setYaw(0); 
            SwerveManual.pigeonAngle=0;
        }));

        // driverGamepad.getButtonB().whilePressed(new MoveBallsToShooter());

        // driverGamepad.getLeftDPadButton().whenPressed(new ZeroHood());
        driverGamepad.getLeftDPadButton().whenPressed(new ZeroHood());

        driverGamepad.getDownDPadButton().whilePressed(new InstantCommand(() -> SmartDashboard.putNumber("desired hood pos", 2)));
        driverGamepad.getRightDPadButton().whilePressed(new InstantCommand(() -> SmartDashboard.putNumber("desired hood pos",14)));
        driverGamepad.getUpDPadButton().whilePressed(new InstantCommand(() -> SmartDashboard.putNumber("desired hood pos", 21)));
        // driverGamepad.getButtonBumperLeft().whenHeld(new MoveBallsToShooter());
        // driverGamepad.getButtonX().whilePressed(new SetIntakeUp());
        // wrap non-commands in lambda but just regular instantiation for commands
        driverGamepad.getButtonA().whenPressed(Autons.THREE_BALL_AUTO);
        // driverGamepad.getButtonA().whenPressed(new SequentialCommandGroup(Trajectories.threepoints.get(0),Trajectories.threepoints.get(1)));
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
