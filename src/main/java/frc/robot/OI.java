package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.Autons;
import frc.robot.auto.Trajectories;
import frc.robot.auto.TurnInPlace;
import frc.robot.commands.climber.SetClimberPosition;
import frc.robot.commands.climber.ToggleClimber;
import frc.robot.commands.drivetrain.AlignWithLimelight;
import frc.robot.commands.drivetrain.SwerveManual;
import frc.robot.commands.hood.ZeroHood;
import frc.robot.commands.indexer.MoveBallsToShooter;
import frc.robot.commands.intake.SetIntakeUp;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.shooter.ShootAgainstHub;
import frc.robot.commands.shooter.ShootWithLimelight;
import frc.robot.commands.shooter.ShooterManual;
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
        ((XboxGamepad)operatorGamepad).getButtonTriggerRight().whilePressed(new MoveBallsToShooter());
        operatorGamepad.getButtonBumperRight().whilePressed(new ShootAgainstHub());
        operatorGamepad.getButtonBumperLeft().whilePressed(new ShootWithLimelight());
        driverGamepad.getButtonY().whenPressed(new ToggleIntake());
        driverGamepad.getButtonStart().whenPressed(new InstantCommand(() -> {
            Drivetrain.getInstance().getPigeon().setYaw(0);
            SwerveManual.pigeonAngle = 0;
        }));
        driverGamepad.getButtonSelect().whenPressed(new ZeroHood());
        operatorGamepad.getLeftDPadButton().whenPressed(new InstantCommand(() -> 
            {
                Climber.getInstance().getClimberMaster().setSelectedSensorPosition(0);
                Climber.getInstance().isZeroed = true;
                Climber.getInstance().getClimberMaster().configReverseSoftLimitEnable(false);
            }));
        operatorGamepad.getUpDPadButton().whenPressed(new SetClimberPosition(Climber.UP_HEIGHT, 0.5, false));
        operatorGamepad.getDownDPadButton().whenPressed(new SetClimberPosition(Climber.DOWN_HEIGHT, 0.7, true));
        operatorGamepad.getRightDPadButton().whenPressed(new SetClimberPosition(Climber.ON_BAR_HEIGHT, 0.3, false));
        operatorGamepad.getButtonY().whenPressed(new ToggleClimber());
        operatorGamepad.getButtonX().whilePressed(new TurnInPlace(90));
        operatorGamepad.getButtonA().whilePressed(Autons.THREE_BALL_AUTO);
        // operatorGamepad.getButtonB().whenPressed(Trajectories.threeBallAuto.get(0));
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
