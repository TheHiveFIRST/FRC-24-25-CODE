
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

  // Controllers
  private final Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  private final Joystick m_operatorController = new Joystick(OIConstants.kOperatorControllerPort);

  // Subsystems
  private final StingerSubsystem m_StingerSubsystem = new StingerSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final LEDSubsystem m_LedSubsystem = new LEDSubsystem();
<<<<<<< HEAD
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();  // Ensuring DriveSubsystem is properly initialized

  // Variables
  private double setPos = 0;
  private double setAng = 0.32;
=======
  // private final DriveSubsystem m_robotDrive = new DriveSubsystem();  // Ensuring DriveSubsystem is properly initialized

  // Variables
  private double setPos = 0;
  private double setAng = 0.295;
>>>>>>> ddeba96e4c330deb0ce002349632e9bda18d9420
  private final DigitalInput limitSwitch = new DigitalInput(2);
  private final Timer m_timer = new Timer();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();
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
<<<<<<< HEAD
    System.out.println("Autonomous Timer: " + m_timer.get());  // Debug log

    if (m_timer.get() < 3.0) {
      m_robotDrive.drive(0, 0.5, 0, true);
      System.out.println("Moving Forward...");
    } else {
      m_robotDrive.drive(0, 0, 0, true); // Stop movement
      System.out.println("Stopping...");

      if (m_timer.get() > 3.0) {
        m_StingerSubsystem.setIntakePower(0.5);
        delayTimer(3);
        m_timer.stop();
      }
    }
  }

  /** Schedules a path-following command */
  public void runPath() {
    Command pathCommand = new PathPlannerAuto("Ishana Path");
    if (pathCommand != null) {
      pathCommand.schedule();
      System.out.println("Running Path: Ishana Path");
    } else {
      System.out.println("Failed to schedule Path");
    }
  }

=======
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

>>>>>>> ddeba96e4c330deb0ce002349632e9bda18d9420
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    handleOperatorControls();
  }

  private void handleOperatorControls() {
    if (m_operatorController.getRawButtonPressed(1)) {
      setElevatorAndPivot(0.1, 0.5);
    } else if (m_operatorController.getRawButtonPressed(2)) {
      setElevatorAndPivot(12.5, 0.5);
    } else if (m_operatorController.getRawButtonPressed(4)) {
      setElevatorAndPivot(22, 0.5);
    } else if (m_operatorController.getRawButtonPressed(3)) {
      setElevatorAndPivot(33.3, 0.47);
    } else if (m_operatorController.getRawButtonReleased(1) || m_operatorController.getRawButtonReleased(2) || m_operatorController.getRawButtonReleased(3) || m_operatorController.getRawButtonReleased(4)) {
<<<<<<< HEAD
      setAng = 0.32; // Reset pivot angle
=======
      setAng = 0.295; // Reset pivot angle
>>>>>>> ddeba96e4c330deb0ce002349632e9bda18d9420
    }

    if (m_operatorController.getRawButtonPressed(5)) {
      m_StingerSubsystem.setIntakePower(0.3);
      m_LedSubsystem.setPattern(-0.99);
    } else if (m_operatorController.getRawButtonPressed(6)) {
      m_StingerSubsystem.setIntakePower(-0.5);
      m_LedSubsystem.setPattern(0.57);
    } else if (m_operatorController.getRawButtonReleased(5) || m_operatorController.getRawButtonReleased(6)) {
      m_StingerSubsystem.setIntakePower(0);
      m_LedSubsystem.setPattern(0.41);
    }

    if (m_operatorController.getRawButtonPressed(7)) {
      setElevatorAndPivot(13, 0.45);
    } else if (m_operatorController.getRawButtonPressed(8)) {
      setElevatorAndPivot(25, 0.45);
    }

    m_ElevatorSubsystem.elevatorPIDControl(setPos);
    m_ElevatorSubsystem.elevatorPIDSetPower();
    m_StingerSubsystem.PivotPIDControl(setAng);
    m_StingerSubsystem.PivotPIDSetPower();

    if (limitSwitch.get()) {
      m_ElevatorSubsystem.resetEncoder();
    }
  }

  private void setElevatorAndPivot(double position, double angle) {
    setPos = position;
    setAng = angle;
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
<<<<<<< HEAD
}
=======
}
>>>>>>> ddeba96e4c330deb0ce002349632e9bda18d9420
