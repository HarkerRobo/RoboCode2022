package frc.robot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.Trajectories;
import frc.robot.commands.drivetrain.HSSwerveDriveController;
import frc.robot.commands.drivetrain.SwerveManual;
import frc.robot.commands.indexer.MoveBallsToShooter;
import frc.robot.commands.intake.IntakeManual;
import frc.robot.commands.intake.ToggleIntake;
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

    public static final double DEADBAND = 0.1;

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
        // driverGamepad.getButtonA().whilePressed(new ShootWithVelocity(15));
        // driverGamepad.getButtonA().whilePressed(new InstantCommand(Drivetrain.getInstance()::toggleFieldCentric));
        // driverGamepad.getButtonB().whilePressed(new IntakeManual(0.3));
        driverGamepad.getButtonY().whilePressed(new ToggleIntake());
        // driverGamepad.getButtonA().whilePressed(new SetIntakeDown());
        driverGamepad.getButtonX().whenPressed(new InstantCommand(() -> {
            Drivetrain.getInstance().getPigeon().zeroGyroBiasNow(); 
            SwerveManual.pigeonAngle=0;
        }));
        // driverGamepad.getButtonBumperLeft().whenHeld(new MoveBallsToShooter());
        // driverGamepad.getButtonX().whilePressed(new SetIntakeUp());
        // wrap non-commands in lambda but just regular instantiation for commands
        // driverGamepad.getButtonA().whenPressed(new SequentialCommandGroup(Trajectories.fiveBallAuto.get(0),Trajectories.fiveBallAuto.get(1),Trajectories.fiveBallAuto.get(2),Trajectories.fiveBallAuto.get(3)));
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
