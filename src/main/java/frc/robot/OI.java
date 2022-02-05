package frc.robot;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.intake.IntakeManual;
import frc.robot.commands.shooter.ShootWithVelocity;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import harkerrobolib.commands.CallMethodCommand;
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
        driverGamepad.getButtonA().whilePressed(new InstantCommand(Drivetrain.getInstance()::toggleFieldCentric));
        driverGamepad.getButtonB().whilePressed(new IntakeManual(0.3));
        // wrap non-commands in lambda but just regular instantiation for commands
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
