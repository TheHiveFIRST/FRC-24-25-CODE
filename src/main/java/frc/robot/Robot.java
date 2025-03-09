

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
import  com.pathplanner.lib.commands.PathPlannerAuto;
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
  // private final DriveSubsystem m_robotDrive = new DriveSubsystem();  // Ensuring DriveSubsystem is properly initialized

  // Variables
  private double setPos = 0;
  private double setAng = 0.295;
  private final DigitalInput limitSwitch = new DigitalInput(2);
  private final Timer m_timer = new Timer();

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  Joystick m_operatorController = new Joystick(OIConstants.kOperatorControllerPort);

  StingerSubsystem m_StingerSubsystem = new StingerSubsystem();
  ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  
  LEDSubsystem m_LedSubsystem = new LEDSubsystem();
  
  double setPos;
  double setAng = 0.32;

  DigitalInput limitSwitch = new DigitalInput(2);
 
  // private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private Timer m_timer = new Timer();
  




  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();
   // CameraServer.startAutomaticCapture(); 
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

   // m_ElevatorSubsystem.encoderGetValue();
   // m_StingerSubsystem.encoderGetValue();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
  

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      System.out.println("Autonomous Command Scheduled");
    } else {
      System.out.println("No Autonomous Command Found");
    }

    m_timer.reset();
    m_timer.start();  // Ensure timer is running
  }
  // public void startTimer(){
  //   timer.reset();
  //   timer.start();

  // }

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

    // if (m_timer.get() < 3.0) {
    //   m_robotDrive.drive(0,0.5,0,true);
    //   System.out.println("Moving Forward...");
    // } else {
    //   // Stop movement after 3 seconds
    //   System.out.println("Stopping...");}


    //   if (m_timer.get() > 3){
    //     m_StingerSubsystem.setIntakePower(0.5);
    //     delayTimer(3);
    //     m_timer.stop();


    //   }


  }
   
  

  

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
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
      setAng = 0.295; // Reset pivot angle
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
  //     if (m_operatorController.getRawButtonPressed(1)){
  //       setPos = 0.1;
  //       new WaitCommand(.5);
  //       setAng = 0.5;
  //     }
  //     else if (m_operatorController.getRawButtonPressed(2)){
  //       setPos = 12.5;
  //       new WaitCommand(.5);
  //       setAng = 0.5;
  //     }
  //     else if (m_operatorController.getRawButtonPressed(4)){
  //       setPos = 22;
  //       new WaitCommand(0.5);
  //       setAng = 0.5;
  //     }
  //     if (m_operatorController.getRawButtonPressed(3)){
  //       setPos = 33.3;
  //       new WaitCommand(0.5);
  //       setAng = 0.47;
  //     }
  //     else if (m_operatorController.getRawButtonReleased(1) || m_operatorController.getRawButtonReleased(2) || m_operatorController.getRawButtonReleased(3) || m_operatorController.getRawButtonReleased(4)){
  //       setAng = 0.32; //default/source angle 
  //     }
      if (m_operatorController.getRawButtonPressed(1)){
        setPos = 0.1;
        new WaitCommand(.5);
        setAng = 0.5;
      }
      else if (m_operatorController.getRawButtonPressed(2)){
        setPos = 9.1; // L2 change 
        new WaitCommand(.5);
        setAng = 0.5;
      }
      else if (m_operatorController.getRawButtonPressed(4)){
        setPos = 18.6;
        new WaitCommand(0.5);
        setAng = 0.5;
      }
      if (m_operatorController.getRawButtonPressed(3)){
        setPos = 32.3;
        new WaitCommand(0.5);
        setAng = 0.53;
      }
      else if (m_operatorController.getRawButtonReleased(1) || m_operatorController.getRawButtonReleased(2) || m_operatorController.getRawButtonReleased(3) || m_operatorController.getRawButtonReleased(4)){
        setAng = 0.27; //default/source angle 

      }
  //     if (m_operatorController.getRawButtonPressed(5)){
  //       m_StingerSubsystem.setIntakePower(0.3);
  //       m_LedSubsystem.setPattern(-0.99);
  //     }
  //     else if (m_operatorController.getRawButtonPressed(6)){
  //       m_StingerSubsystem.setIntakePower(-0.5);
  //       m_LedSubsystem.setPattern(0.57);
  //     }
  //     else if (m_operatorController.getRawButtonReleased(5) || m_operatorController.getRawButtonReleased(6)) {
  //       m_StingerSubsystem.setIntakePower(0);
  //       m_LedSubsystem.setPattern(0.41);
  //     }
  //     else if (m_operatorController.getRawButtonPressed(  7)){
  //       setPos = 13;
  //       new WaitCommand(0.5);
  //       setAng = 0.45;
  //     }
  //     else if (m_operatorController.getRawButtonPressed(8)){
  //       setPos = 25;
  //       new WaitCommand(0.5);
  //       setAng = 0.45;
  //     }


    
  //     m_ElevatorSubsystem.elevatorPIDControl(setPos);
  //     m_ElevatorSubsystem.elevatorPIDSetPower();
  //     //calculate and set PID for motor

  //     m_StingerSubsystem.PivotPIDControl(setAng);
  //     m_StingerSubsystem.PivotPIDSetPower(); 
  //     //calculate and set
     
  //     if (limitSwitch.get()){
  //       m_ElevatorSubsystem.resetEncoder();
  //     }
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

  public void testPeriodic(){}

    public void startTimer() {
      m_timer.restart(); // Reset the timer to 0
      m_timer.start(); // Start the timer
   }
  
   public double getElapsedTime() {
       return m_timer.get(); // Get elapsed time in seconds
   }
  
  public void delayTimer(int seconds){
 //   m_timer.delay(seconds);
  }
  
  public void stopTimer() {
    m_timer.stop(); // Stop the timer
   }
  }