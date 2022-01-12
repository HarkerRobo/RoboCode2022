package frc.robot;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.auto.Autons;
import frc.robot.auto.Trajectories;
import frc.robot.commands.drivetrain.HSSwerveDriveController;
import frc.robot.commands.drivetrain.SwerveManualHeadingControl;
import frc.robot.commands.drivetrain.SwerveTranslationAlign;
import frc.robot.commands.intake.IntakeAutonControlForward;
import frc.robot.commands.intake.MoveBallsToShooter;
import frc.robot.commands.shooter.ShooterVelocityManual;
import frc.robot.commands.spine.Jumble;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import harkerrobolib.commands.CallMethodCommand;
import harkerrobolib.wrappers.HSGamepad;
import harkerrobolib.wrappers.XboxGamepad;


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

    private void initBindings() {
        operatorGamepad.getButtonA().whenPressed(new InstantCommand(() -> {
            Intake.getInstance().invertSolenoid();
        }, Intake.getInstance()));
        
        // driverGamepad.getButtonBumperRight().whilePressed(new ParallelCommandGroup(
        //     new MoveBallsToShooter(), new ShooterManual()
        // ));

        driverGamepad.getButtonY().whilePressed(new SwerveTranslationAlign());

        operatorGamepad.getButtonBumperRight().whilePressed(new ParallelCommandGroup(
            new ShooterVelocityManual(78.5),
            new MoveBallsToShooter()
        ));
        // driverGamepad.getRightDPadButton().whenPressed(new ParallelCommandGroup(
        //     Autons.autoPath1
        // ));
        operatorGamepad.getButtonA().whenPressed(new InstantCommand(() -> Intake.getInstance().invertSolenoid(), Intake.getInstance()));
        operatorGamepad.getButtonStart().whilePressed(new ShooterVelocityManual(78.5));


        driverGamepad.getButtonSelect().whenPressed(new InstantCommand(() -> {Drivetrain.getInstance().getPigeon().addFusedHeading(-63.9886 * Drivetrain.getInstance().getPigeon().getFusedHeading()); 
            SwerveManualHeadingControl.headingFlag = false;
            SwerveManualHeadingControl.flag = false;}));
    }
    
    public HSGamepad getDriverGamepad(){
        return driverGamepad;
    }
    public HSGamepad getOperatorGamepad(){
        return operatorGamepad;
    }

    public static OI getInstance(){
        if(oi==null)
            oi=new OI();
        return oi;
    }
}
