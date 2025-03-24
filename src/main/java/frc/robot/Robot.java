


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.StingerSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.Timer;

/**
 * Main robot class extending TimedRobot.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;


  // Variables

  private final Timer m_timer = new Timer();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    //meraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      System.out.println("Autonomous Command Scheduled");
    } else {
      System.out.println("No Autonomous Command Found");
    }

    m_timer.reset();
    m_timer.start();  // Ensure timer is running
  }

  /** Runs the autonomous sequence */
  @Override
  public void autonomousPeriodic() {
    // System.out.println("Autonomous Timer: " + m_timer.get());  // Debug log

    // if (m_timer.get() < 3.0) {
    //   m_robotDrive.drive(0, 0.5, 0, true);
    //   System.out.println("Moving Forward...");
    // } else {
    //   m_robotDrive.drive(0, 0, 0, true); // Stop movement
    //   System.out.println("Stopping...");

    //   if (m_timer.get() > 3.0) {
    //     m_StingerSubsystem.setIntakePower(0.5);
    //     delayTimer(3);
    //     m_timer.stop();
    //   }
    // }
  }

  /** Schedules a path-following command */
  // public void runPath() {
  //   Command pathCommand = new PathPlannerAuto("Ishana Path");
  //   if (pathCommand != null) {
  //     pathCommand.schedule();
  //     System.out.println("Running Path: Ishana Path");
  //   } else {
  //     System.out.println("Failed to schedule Path");
  //   }
  // }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
   }
  

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  /** Delays execution for a set time */
  public void delayTimer(int seconds) {
    m_timer.delay(seconds);
  }
}
