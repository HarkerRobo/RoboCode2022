
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
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
  private Timer pitchVel;
  private boolean wasAuto = false;

  private Command auto = Autons.FIVE_BALL_AUTO;
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
    pitchVel = new Timer();
    pitchVel.reset();
    pitchVel.start();
    SmartDashboard.putData(field);
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
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
    // DoubleSolenoid pressure = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 4);
    // pressure.set(DoubleSolenoid.Value.kForward);
    NetworkTableInstance.getDefault().setUpdateRate(0.02);
    
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
    field.setRobotPose(new Pose2d(robotPose.getX(), robotPose.getY(), robotPose.getRotation()));
    if(pitchVel.hasElapsed(0.06)) {
      pitchVel.reset();
      Drivetrain.getInstance().updatePitchVel();
    }
    SmartDashboard.putData(Drivetrain.getInstance());
    SmartDashboard.putData(Climber.getInstance());
    SmartDashboard.putData(Intake.getInstance());
    SmartDashboard.putData(Indexer.getInstance());
    SmartDashboard.putData(Shooter.getInstance());
  }

  @Override
  public void autonomousInit() {
    Limelight.setLEDS(true);
    Limelight.update();
    auto.schedule();
  }

  @Override
  public void autonomousPeriodic() {
    Limelight.update();
    wasAuto = true;
    pd.setSwitchableChannel(true);
  }

  /**
   * This function is called once when teleop is enabled.
   */
  @Override
  public void teleopInit() {
    if(wasAuto) {
      wasAuto = false;
      if(auto == Autons.THREE_BALL_AUTO) {
        // Drivetrain.getInstance().getPigeon().setYaw() 
      }
    }
    Limelight.setLEDS(true);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    pd.setSwitchableChannel(true);
    Limelight.update();
    // if(Math.random() < 1.0/3000)
    //   CommandScheduler.getInstance().schedule(new ToggleIntake());
  }

  /**
   * This function is called once when the robot is disabled.
   */
  @Override
  public void disabledInit() {
    Limelight.setLEDS(false);
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
