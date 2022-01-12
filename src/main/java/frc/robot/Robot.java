/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.intake.IntakeControl;
import frc.robot.commands.drivetrain.SwerveManual;
import frc.robot.commands.drivetrain.SwerveManualHeadingControl;
import frc.robot.subsystems.Drivetrain;
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

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    Drivetrain drivetrain = Drivetrain.getInstance();
    drivetrain.getTopLeft().getRotationMotor().setSelectedSensorPosition((Drivetrain.getInstance().getTopLeft().getRotationMotor().getSensorCollection().getPulseWidthRiseToFallUs() - Drivetrain.TL_OFFSET));
    drivetrain.getTopRight().getRotationMotor().setSelectedSensorPosition((Drivetrain.getInstance().getTopRight().getRotationMotor().getSensorCollection().getPulseWidthRiseToFallUs() - Drivetrain.TR_OFFSET));
    drivetrain.getBottomLeft().getRotationMotor().setSelectedSensorPosition((Drivetrain.getInstance().getBottomLeft().getRotationMotor().getSensorCollection().getPulseWidthRiseToFallUs() - Drivetrain.BL_OFFSET));
    drivetrain.getBottomRight().getRotationMotor().setSelectedSensorPosition((Drivetrain.getInstance().getBottomRight().getRotationMotor().getSensorCollection().getPulseWidthRiseToFallUs() - Drivetrain.BR_OFFSET));
    CommandScheduler.getInstance().setDefaultCommand(Drivetrain.getInstance(), new SwerveManual());
    CommandScheduler.getInstance().setDefaultCommand(Intake.getInstance(), new IntakeControl());
    // Indexer.getInstance().getSolenoid().set(Indexer.BLOCKER_CLOSED);
    // CommandScheduler.getInstance().setDefaultCommand(Shooter.getInstance(), new ShooterManual());
    // CommandScheduler.getInstance().setDefaultCommand(Indexer.getInstance(), new LinearManual());

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
    Drivetrain drivetrain = Drivetrain.getInstance();

    drivetrain.getOdometry().update(
      Rotation2d.fromDegrees(drivetrain.getPigeon().getFusedHeading()), 
      drivetrain.getTopLeft().getState(),
      drivetrain.getTopRight().getState(), 
      drivetrain.getBottomLeft().getState(), 
      drivetrain.getBottomRight().getState()
    );

    
    
    SmartDashboard.putNumber("angle pos tl pulse width", drivetrain.getTopLeft().getRotationMotor().getSensorCollection().getPulseWidthRiseToFallUs());
    SmartDashboard.putNumber("angle pos tr pulse width", drivetrain.getTopRight().getRotationMotor().getSensorCollection().getPulseWidthRiseToFallUs());
    SmartDashboard.putNumber("angle pos bl pulse width", drivetrain.getBottomLeft().getRotationMotor().getSensorCollection().getPulseWidthRiseToFallUs());
    SmartDashboard.putNumber("angle pos br pulse width", drivetrain.getBottomRight().getRotationMotor().getSensorCollection().getPulseWidthRiseToFallUs());

    SmartDashboard.putNumber("ODOMOTER X", drivetrain.getOdometry().getPoseMeters().getX());
    SmartDashboard.putNumber("ODOMOTER Y", drivetrain.getOdometry().getPoseMeters().getY());
    SmartDashboard.putNumber("ODOMOTER ANGLE", drivetrain.getOdometry().getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("STATE SPEED", drivetrain.getBottomLeft().getState().speedMetersPerSecond);
    SmartDashboard.putNumber("SENSOR SPEEDs", drivetrain.getBottomLeft().getTranslationMotor().getSelectedSensorVelocity());
    SmartDashboard.putNumber("STATE ANGLE", drivetrain.getBottomLeft().getState().angle.getDegrees());
    

    SmartDashboard.putNumber("angle pos tl", drivetrain.getTopLeft().getRotationMotor().getSelectedSensorPosition());
    SmartDashboard.putNumber("angle pos tr", drivetrain.getTopRight().getRotationMotor().getSelectedSensorPosition());
    SmartDashboard.putNumber("angle pos bl", drivetrain.getBottomLeft().getRotationMotor().getSelectedSensorPosition());
    SmartDashboard.putNumber("angle pos br", drivetrain.getBottomRight().getRotationMotor().getSelectedSensorPosition());

    SmartDashboard.putNumber("Pigeon Heading", drivetrain.getPigeon().getFusedHeading());

    // SmartDashboard.putString("cd color spinner current color", Spinner.getInstance().getCurrentColor().toString());
    SmartDashboard.putString("cd color spinner desired color", DriverStation.getInstance().getGameSpecificMessage());
    // SmartDashboard.putBoolean("cd hood sol", Shooter.getInstance().getSolenoid().get() == Value.kReverse);
    SmartDashboard.putBoolean("cd intake sol", Intake.getInstance().getSolenoid().get() == Value.kForward);
    SmartDashboard.putNumber("cd pigeon angle", Drivetrain.getInstance().getPigeon().getFusedHeading());
    // SmartDashboard.putString("cd current auton", isTeleop ? Autons.curAuton.toString() : "Teleop Running");
    SmartDashboard.putString("cd current auton", "Teleop Running");
    SmartDashboard.putBoolean("cd shooter isStalling", Shooter.getInstance().isStalling());
    SmartDashboard.putBoolean("cd intake isStalling", Intake.getInstance().isStalling());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    Limelight.setLEDS(false);
    // m_autoSelected = m_chooser.getSelected();
    // // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    // System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // switch (m_autoSelected) {
    //   case kCustomAuto:
    //     // Put custom auto code here
    //     break;
    //   case kDefaultAuto:
    //   default:
    //     // Put default auto code here
    //     break;
    // }
  }

  /**
   * This function is called once when teleop is enabled.
   */
  @Override
  public void teleopInit() {
    Limelight.setLEDS(true);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
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
