
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.drivetrain.SwerveManualPercentOutput;
import frc.robot.commands.intake.IntakeManual;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

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
   * used for any initialization code. It corrects the starting rotation motors
   * using CAN coders but doesn't move the motors, only setting their sensor positions.
   */
  @Override
  public void robotInit() {
    
    // default commands are commands that are always running on the robot
    CommandScheduler.getInstance().setDefaultCommand(Drivetrain.getInstance(), new SwerveManualPercentOutput());
    CommandScheduler.getInstance().setDefaultCommand(Intake.getInstance(), new IntakeManual());
    Drivetrain.getInstance().readCANCoders();
    // CommandScheduler.getInstance().setDefaultCommand(Shooter.getInstance(), new ShooterManual());
    // OI.getInstance();
    SmartDashboard.putNumber("desired velocity", 0);
    SmartDashboard.putNumber("desired angle", 0);
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

    SmartDashboard.putNumber("tl abs", Drivetrain.getInstance().getTopLeft().getCanCoder().getAbsolutePosition());
    SmartDashboard.putNumber("tr abs", Drivetrain.getInstance().getTopRight().getCanCoder().getAbsolutePosition());
    SmartDashboard.putNumber("bl abs", Drivetrain.getInstance().getBottomLeft().getCanCoder().getAbsolutePosition());
    SmartDashboard.putNumber("br abs", Drivetrain.getInstance().getBottomRight().getCanCoder().getAbsolutePosition());

    SmartDashboard.putNumber("tl angle", Drivetrain.getInstance().getTopLeft().getRotationAngle());
    SmartDashboard.putNumber("tr angle", Drivetrain.getInstance().getTopRight().getRotationAngle());
    SmartDashboard.putNumber("bl angle", Drivetrain.getInstance().getBottomLeft().getRotationAngle());
    SmartDashboard.putNumber("br angle", Drivetrain.getInstance().getBottomRight().getRotationAngle());

    SmartDashboard.putNumber("bottom left encoder tick", Drivetrain.getInstance().getBottomLeft().getRotationMotor().getSelectedSensorPosition());
    // SmartDashboard.putNumber("current vel", Shooter.getInstance().getMaster().getSelectedSensorVelocity() * 10 / 2048 * 4 * Math.PI * 2.54 / 100);
  }

  /**
   * This function is called once when teleop is enabled.
   */
  @Override
  public void teleopInit() {
   
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
  }

  /**
   * This function is called periodically when disabled.
   */
  @Override
  public void disabledPeriodic() {
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
