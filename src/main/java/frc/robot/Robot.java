
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.Autons;
import frc.robot.commands.climber.ClimberManual;
import frc.robot.commands.drivetrain.SwerveManual;
import frc.robot.commands.hood.HoodManual;
import frc.robot.commands.indexer.IndexerManual;
import frc.robot.commands.indexer.MoveBallsToShooter;
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
  private SendableChooser<CommandBase> autonChooser;
  public static boolean thirdPoint = false;

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
    UsbCamera cam = CameraServer.startAutomaticCapture();
    System.out.println(cam.setResolution(80, 60));
    cam.setFPS(10);
    // CameraServer.getServer().setConfigJson("{ \"fps\": 30, \"height\": 120, \"pixel format\": \"mjpeg\", \"properties\": [ { \"name\": \"connect_verbose\", \"value\": 1 }, { \"name\": \"raw_brightness\", \"value\": 0 }, { \"name\": \"brightness\", \"value\": 50 }, { \"name\": \"raw_contrast\", \"value\": 32 }, { \"name\": \"contrast\", \"value\": 50 }, { \"name\": \"raw_saturation\", \"value\": 60 }, { \"name\": \"saturation\", \"value\": 46 }, { \"name\": \"raw_hue\", \"value\": 0 }, { \"name\": \"hue\", \"value\": 50 }, { \"name\": \"white_balance_temperature_auto\", \"value\": true }, { \"name\": \"gamma\", \"value\": 100 }, { \"name\": \"raw_gain\", \"value\": 0 }, { \"name\": \"gain\", \"value\": 0 }, { \"name\": \"power_line_frequency\", \"value\": 1 }, { \"name\": \"white_balance_temperature\", \"value\": 4600 }, { \"name\": \"raw_sharpness\", \"value\": 2 }, { \"name\": \"sharpness\", \"value\": 33 }, { \"name\": \"backlight_compensation\", \"value\": 1 }, { \"name\": \"exposure_auto\", \"value\": 3 }, { \"name\": \"raw_exposure_absolute\", \"value\": 157 }, { \"name\": \"exposure_absolute\", \"value\": 3 }, { \"name\": \"exposure_auto_priority\", \"value\": true } ], \"width\": 160 }");

    // CommandScheduler.getInstance().setDefaultCommand(Shooter.getInstance(), new ShooterManual());
    // OI.getInstance();
    // DoubleSolenoid pressure = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 4);
    // pressure.set(DoubleSolenoid.Value.kForward);
    autonChooser = new SendableChooser<>();
    autonChooser.setDefaultOption("Five Ball Auton", Autons.FIVE_BALL_AUTO);
    // autonChooser.addOption("Three Ball Auton", Autons.THREE_BALL_AUTO);
    autonChooser.addOption("Two Ball Auton", Autons.TWO_BALL_AUTO);
    // autonChooser.addOption("One Ball Auton", Autons.ONE_BALL_AUTO);
    SmartDashboard.putData("Auton Selector", autonChooser);
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

    Drivetrain.getInstance().getPoseEstimator().update(
      Drivetrain.getInstance().getHeadingRotation(), 
      Drivetrain.getInstance().getTopLeft().getState(),
      Drivetrain.getInstance().getTopRight().getState(), 
      Drivetrain.getInstance().getBottomLeft().getState(), 
      Drivetrain.getInstance().getBottomRight().getState()
    );
    // Pose2d robotPose = Drivetrain.getInstance().getOdometry().getPoseMeters();

    if(Limelight.isTargetVisible()) {
      Limelight.update();
      Translation2d HUB = new Translation2d(8.2296, 4.1148);

      double angle = Drivetrain.getInstance().getHeading() - Limelight.getTx();
      double distance = MoveBallsToShooter.HUB_RADIUS + Limelight.getDistance();
      Pose2d newPos = new Pose2d(HUB.getX() - Math.cos(Math.toRadians(angle)) * distance, 
        HUB.getY() - Math.sin(Math.toRadians(angle)) * distance, Drivetrain.getInstance().getHeadingRotation());
      // Drivetrain.getInstance().getOdometry().resetPosition(newPos, Drivetrain.getInstance().getHeadingRotation());
      Drivetrain.getInstance().getPoseEstimator().addVisionMeasurement(newPos, Timer.getFPGATimestamp() - Limelight.getTl()/1000 - 0.02*2.5);
      // Limelight.getTl()/1000.0;
      // field.setRobotPose(Drivetrain.getInstance().getPoseEstimator().getEstimatedPosition());
    }
    field.setRobotPose(Drivetrain.getInstance().getOdometry().getPoseMeters());
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
    autonChooser.getSelected().schedule();
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
      Drivetrain.getInstance().getPigeon().setYaw(Drivetrain.getInstance().getOdometry().getPoseMeters().getRotation().getDegrees());
      Drivetrain.getInstance().getOdometry().resetPosition(Drivetrain.getInstance().getOdometry().getPoseMeters(), Drivetrain.getInstance().getOdometry().getPoseMeters().getRotation());
      SwerveManual.pigeonAngle = 0;
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
    // Limelight.update();
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
