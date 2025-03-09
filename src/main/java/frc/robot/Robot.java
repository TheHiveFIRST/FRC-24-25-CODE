// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.button.POVButton;
//import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.StingerSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import  com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.Timer;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

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
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
   // CameraServer.startAutomaticCapture(); 
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    CommandScheduler.getInstance().run();
   // m_ElevatorSubsystem.encoderGetValue();
   // m_StingerSubsystem.encoderGetValue();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
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
    }
  }

  // public void startTimer(){
  //   timer.reset();
  //   timer.start();

  // }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
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

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
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
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
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

