package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.auto.Autons;
import frc.robot.commands.climber.ClimberManual;
import frc.robot.commands.climber.MoveClimbToNextBar;
import frc.robot.commands.climber.SetClimberPosition;
import frc.robot.commands.drivetrain.SwerveManual;
import frc.robot.commands.hood.HoodManual;
import frc.robot.commands.hood.SetHoodFast;
import frc.robot.commands.hood.ZeroHood;
import frc.robot.commands.indexer.MoveBallsToShooter;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.shooter.ShootAgainstHub;
import frc.robot.commands.shooter.ShootWithLimelight;
import frc.robot.subsystems.Climber;
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
        operatorGamepad.getButtonBumperRight().whilePressed(new ParallelCommandGroup(new ShootAgainstHub(), new MoveBallsToShooter(false)));
        operatorGamepad.getButtonBumperLeft().whilePressed(new ParallelCommandGroup(new ShootWithLimelight(), new MoveBallsToShooter(false)));
        driverGamepad.getButtonBumperRight().whilePressed(new ParallelCommandGroup(new ShootWithLimelight(), new MoveBallsToShooter(!RobotMap.DEMO_MODE)));

        driverGamepad.getButtonY().whenPressed(new SetClimberPosition(Climber.DOWN_HEIGHT, 0.15));
        driverGamepad.getButtonStart().whenPressed(new InstantCommand(() -> {
            Drivetrain.getInstance().getPigeon().setYaw(0);
            SwerveManual.pigeonAngle = 0;
            if (RobotMap.DEMO_MODE) {
                Drivetrain.getInstance().getOdometry().resetPosition(new Pose2d(0, 0, Drivetrain.getInstance().getHeadingRotation()), Drivetrain.getInstance().getHeadingRotation());
            }
        }));
        operatorGamepad.getButtonY().whenPressed(new SetHoodFast(20));
        
        driverGamepad.getButtonSelect().whenPressed(new ZeroHood());        
        operatorGamepad.getButtonSelect().whenPressed(new ZeroHood());   
        driverGamepad.getUpDPadButton().whenPressed(new SetClimberPosition(Climber.UP_HEIGHT, ClimberManual.MAGNITUDE_UP));
        driverGamepad.getDownDPadButton().whenPressed(new SetClimberPosition(Climber.DOWN_HEIGHT, 0.6));
        driverGamepad.getRightDPadButton().whenPressed(new MoveClimbToNextBar());
        driverGamepad.getButtonA().whenPressed(new InstantCommand(Climber.getInstance()::toggleClimber));
        driverGamepad.getLeftDPadButton().whenPressed(new SetClimberPosition(2*Climber.UP_HEIGHT / 3 , 0.6));
        operatorGamepad.getButtonB().whenPressed(new InstantCommand(() -> HoodManual.downMode = !HoodManual.downMode));
        operatorGamepad.getRightDPadButton().whenPressed(new InstantCommand(()->ShootWithLimelight.velocityOffset += 0.2));
        operatorGamepad.getLeftDPadButton().whenPressed(new InstantCommand(()->ShootWithLimelight.velocityOffset -= 0.2));
        // operatorGamepad.getButtonX().whilePressed(new TurnInPlace(90));
        if(!RobotMap.DEMO_MODE)
            driverGamepad.getButtonX().whenPressed(Drivetrain.getInstance()::toggleFieldCentric);
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
