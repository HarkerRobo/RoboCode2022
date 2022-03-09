
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.Autons;
import frc.robot.commands.climber.ClimberManual;
import frc.robot.commands.drivetrain.SwerveManual;
import frc.robot.commands.hood.HoodManual;
import frc.robot.commands.indexer.IndexerManual;
import frc.robot.commands.intake.IntakeManual;
import frc.robot.commands.shooter.ShooterManual;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Limelight;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  Field2d field;
  private PowerDistribution pd;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code. It corrects the starting rotation motors
   * using CAN coders but doesn't move the motors, only setting their sensor positions.
   */
  @Override
  public void robotInit() {
    Limelight.setLEDS(false);
    pd = new PowerDistribution();
    pd.setSwitchableChannel(false);
    field = new Field2d();
    SmartDashboard.putData(field);
    // new Compressor(PneumaticsModuleType.REVPH).disable();
    // default commands are commands that are always running on the robot
    CommandScheduler.getInstance().setDefaultCommand(Drivetrain.getInstance(), new SwerveManual());
    CommandScheduler.getInstance().setDefaultCommand(Indexer.getInstance(), new IndexerManual());
    CommandScheduler.getInstance().setDefaultCommand(Intake.getInstance(), new IntakeManual());
    CommandScheduler.getInstance().setDefaultCommand(Hood.getInstance(), new HoodManual());
    CommandScheduler.getInstance().setDefaultCommand(Climber.getInstance(), new ClimberManual());

    Drivetrain.getInstance().readCANCoders();
    new Notifier(()->Drivetrain.getInstance().readCANCoders()).startSingle(5);
    // CommandScheduler.getInstance().setDefaultCommand(Shooter.getInstance(), new ShooterManual());
    // OI.getInstance();
    SmartDashboard.putNumber("desired velocity", 0);
    SmartDashboard.putNumber("desired hood angle", 0.5);
    SmartDashboard.putNumber("desired angle", 90);
    SmartDashboard.putNumber("intake RPS", 0.1);
    SmartDashboard.putNumber("desired hood pos", 0);
    SmartDashboard.putNumber("hood P", HoodManual.HOOD_KP);
    SmartDashboard.putNumber("hood I", HoodManual.HOOD_KI);
    SmartDashboard.putNumber("hood D", HoodManual.HOOD_KD);
    SmartDashboard.putNumber("hood izone", HoodManual.HOOD_IZONE);
    // DoubleSolenoid pressure = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 4);
    // pressure.set(DoubleSolenoid.Value.kForward);
    // NetworkTableInstance.getDefault().setUpdateRate(0.02);
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    Drivetrain.getInstance().getOdometry().update(
      Drivetrain.getInstance().getHeadingRotation(), 
      Drivetrain.getInstance().getTopLeft().getState(),
      Drivetrain.getInstance().getTopRight().getState(), 
      Drivetrain.getInstance().getBottomLeft().getState(), 
      Drivetrain.getInstance().getBottomRight().getState()
    );
    Pose2d robotPose = Drivetrain.getInstance().getOdometry().getPoseMeters();
    field.setRobotPose(new Pose2d(-robotPose.getY(), robotPose.getX(), robotPose.getRotation()));

    SmartDashboard.putNumber("shooter encoder ticks", Shooter.getInstance().getShooterEncoder().get());
    SmartDashboard.putNumber("limelight distance", Limelight.getDistance());
    SmartDashboard.putNumber("odometry x", Drivetrain.getInstance().getOdometry().getPoseMeters().getX());
    SmartDashboard.putNumber("odometry y", Drivetrain.getInstance().getOdometry().getPoseMeters().getY());
    SmartDashboard.putNumber("odometry theta", Drivetrain.getInstance().getOdometry().getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("tl abs", Drivetrain.getInstance().getTopLeft().getCanCoder().getAbsolutePosition());
    SmartDashboard.putNumber("tr abs", Drivetrain.getInstance().getTopRight().getCanCoder().getAbsolutePosition());
    SmartDashboard.putNumber("bl abs", Drivetrain.getInstance().getBottomLeft().getCanCoder().getAbsolutePosition());
    SmartDashboard.putNumber("br abs", Drivetrain.getInstance().getBottomRight().getCanCoder().getAbsolutePosition());
    SmartDashboard.putNumber("limit switch", Climber.getInstance().getClimberMaster().isRevLimitSwitchClosed());
    // SmartDashboard.putNumber("tl angle", Drivetrain.getInstance().getTopLeft().getRotationAngle());
    // SmartDashboard.putNumber("tr angle", Drivetrain.getInstance().getTopRight().getRotationAngle());
    // SmartDashboard.putNumber("bl angle", Drivetrain.getInstance().getBottomLeft().getRotationAngle());
    // SmartDashboard.putNumber("br angle", Drivetrain.getInstance().getBottomRight().getRotationAngle());

    // SmartDashboard.putNumber("bl angle error", Drivetrain.getInstance().getBottomLeft().getRotationAngle());

    SmartDashboard.putNumber("pigeon angle", Drivetrain.getInstance().getHeading());
    // SmartDashboard.putNumber("bottom left angle error", Drivetrain.getInstance().getBottomLeft().getRotationMotor().getClosedLoopError());
    SmartDashboard.putNumber("bottom left control effort", Drivetrain.getInstance().getBottomLeft().getRotationMotor().getMotorOutputPercent());
    SmartDashboard.putNumber("top left speed", Math.abs(Drivetrain.getInstance().getTopLeft().getTranslationVelocity()));
    SmartDashboard.putNumber("target top left speed", Math.abs(SmartDashboard.getNumber("Desired translation speed 0", 0.1)));
    SmartDashboard.putNumber("top left kalman speed", Math.abs(Drivetrain.getInstance().getTopLeft().getTranslationLoop().getVelocity()));
    SmartDashboard.putNumber("top left control effort", Math.abs(Drivetrain.getInstance().getTopLeft().getTranslationMotor().getMotorOutputVoltage()/10));
    SmartDashboard.putNumber("climber pos", Climber.getInstance().getPosition());
    // SmartDashboard.putNumber("hood pos falcon", Hood.getInstance().getHood().getSelectedSensorPosition());
    // SmartDashboard.putNumber("cur hood pid error", Hood.getInstance().getHood().getClosedLoopError());
    // SmartDashboard.putNumber("current vel", Shooter.getInstance().getMaster().getSelectedSensorVelocity() * 10 / 2048 * 4 * Math.PI * 2.54 / 100);
    SmartDashboard.putNumber("hood pos", Hood.getInstance().getHoodPos());
    SmartDashboard.putNumber("bottom r", Indexer.getInstance().getColor().red);
    SmartDashboard.putNumber("bottom g", Indexer.getInstance().getColor().green);
    SmartDashboard.putNumber("bottom b", Indexer.getInstance().getColor().blue);
    SmartDashboard.putNumber("bottom proximity", Indexer.getInstance().getProximity());
    SmartDashboard.putBoolean("bottom occupied", Indexer.getInstance().bottomOccupied());
    SmartDashboard.putBoolean("top occupied", Indexer.getInstance().topOccupied());
    SmartDashboard.putNumber("ll tx", Limelight.getTx());
    SmartDashboard.putNumber("ll distance", Limelight.getDistance());
  }

  @Override
  public void autonomousInit() {
    Limelight.setLEDS(true);
    Limelight.update();
    pd.setSwitchableChannel(true);
    Autons.ONE_BALL_AUTO.schedule();
  }

  /**
   * This function is called once when teleop is enabled.
   */
  @Override
  public void teleopInit() {
    Limelight.setLEDS(true);
    pd.setSwitchableChannel(true);  
    Autons.ONE_BALL_AUTO.cancel();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    // if(Math.random() < 1.0/3000)
    //   CommandScheduler.getInstance().schedule(new ToggleIntake());
  }

  /**
   * This function is called once when the robot is disabled.
   */
  @Override
  public void disabledInit() {
    Limelight.setLEDS(false);
    pd.setSwitchableChannel(false);
  }

  /**
   * This function is called periodically when disabled.
   */
  @Override
  public void disabledPeriodic() {
    Limelight.setLEDS(false);
    pd.setSwitchableChannel(false);
  }

  /**
   * This function is called once when test mode is enabled.
   */
  @Override
  public void testInit() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
