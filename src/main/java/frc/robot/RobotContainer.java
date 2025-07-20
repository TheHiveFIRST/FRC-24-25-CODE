// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToReefTagRelative;
//import frc.robot.commands.OuttakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.StingerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.util.function.IntFunction;

//import java.util.HashMap;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // initialize robot's subsystems
  private final static DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final static LEDSubsystem m_LED = new LEDSubsystem();
  private final static ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final static StingerSubsystem m_stinger = new StingerSubsystem();
  private final static OuttakeSubsystem m_outtake = new OuttakeSubsystem();
  private final static DigitalInput limitSwitch = new DigitalInput(2);

 // private final DigitalInput limitSwitch = new DigitalInput(2);

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  double driveSpeed = 1;
  double SLOW_MODE_MULTIPLIER = 0.5;
  Trigger limitSwitchActivation = new Trigger(limitSwitch::get);

  //intialize the path planner auto command
  private PathPlannerAuto straightPath = new PathPlannerAuto("StraightFromMiddleAuto");

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */  
  public RobotContainer() {  
    //Autonomous Path 
    NamedCommands.registerCommand("shoot", new RunCommand(
      () -> m_outtake.setIntakePower(-0.3,-0.3)));
    NamedCommands.registerCommand("stopmotor", new RunCommand(
      () -> m_outtake.setIntakePower(0,0)));
    NamedCommands.registerCommand("PivotL4", new RunCommand(
      () -> m_stinger.pivotPIDControl(0.5)));
    NamedCommands.registerCommand("ElevatorUp", new RunCommand(
      () -> m_elevator.elevatorPIDControl(32.3)));


    new EventTrigger("ElevatorUp").onTrue(new RunCommand(
      () -> m_elevator.elevatorPIDControl(32.3)));
    
    new EventTrigger("PivotL4").onTrue(new RunCommand(
      () -> m_stinger.pivotPIDControl(0.5)));

    new EventTrigger("shoot").whileTrue(new RunCommand(
      () -> m_outtake.setIntakePower(-0.3,-0.3), m_outtake));

    new EventTrigger("stopmotor").whileTrue(new RunCommand(
      () -> m_outtake.setIntakePower(0,0), m_outtake));  

    
    configureButtonBindings();
    //m_elevator.setDefaultCommand(new RunCommand(()-> m_elevator.elevatorPIDControl(0), m_elevator));
    // m_stinger.setDefaultCommand(new RunCommand(()-> m_stinger.pivotPIDControl(0.3), m_stinger));

    // Configure default commands
    m_outtake.setDefaultCommand(new RunCommand(()-> m_outtake.setIntakePower(0,0), m_outtake));
  

    m_stinger.setDefaultCommand(new RunCommand(() -> m_stinger.wristPivotPIDControl(0.313), m_stinger)); //DEFAULT POSITION FOR WRIST 

    m_LED.setDefaultCommand(new RunCommand(()-> m_LED.setPattern(-0.99), m_LED));

    //Find elevator value 
    m_elevator.setDefaultCommand(new RunCommand(() -> m_elevator.elevatorPIDControl(m_elevator.currentPosition)
     , m_elevator));
    //m_elevator.setDefaultCommand(new RunCommand(() -> {m_elevator.encoderGetValue();
       //System.out.println(m_elevator.encoderGetValue());}
      //, m_elevator));

    m_robotDrive.setDefaultCommand(
      new RunCommand(
      () -> {
              boolean slowMode = m_driverController.getLeftBumperButton(); // Use Left Bumper for slow mode
              double currentDriveSpeed = slowMode ? driveSpeed * SLOW_MODE_MULTIPLIER : driveSpeed;
  
              m_robotDrive.drive(
                  -MathUtil.applyDeadband((m_driverController.getLeftY() * currentDriveSpeed), OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband((m_driverController.getLeftX() * currentDriveSpeed), OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband((m_driverController.getRightX() * currentDriveSpeed), OIConstants.kDriveDeadband),
                   true);
             }, m_robotDrive));
  }

  // public static void runDuringTeleop(){
  //  m_elevator.setDefaultCommand(new RunCommand(()-> m_elevator.elevatorPIDControl(0), m_elevator));
  //  m_stinger.setDefaultCommand(new RunCommand(() -> m_stinger.wristPivotPIDControl(0.27), m_stinger));
  
  // } 

   /* created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    //configure the dpad buttons 
    POVButton dpadUp = new POVButton(m_operatorController, 0);
    POVButton dpadRight = new POVButton(m_operatorController, 90);
    POVButton dpadDown = new POVButton(m_operatorController, 180);
    POVButton dpadLeft = new POVButton(m_operatorController, 270);

    //sets the swerve drive wheels to x locked position
    // new JoystickButton(m_driverController, Button.kX.value)
    //     .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
    
    //TEST WRIST PIVOT
    new JoystickButton(m_driverController, Button.kB.value)
        .whileTrue(new RunCommand(() -> { 
        m_stinger.wristPivotPIDControl(0.206);
        //m_stinger.setPivotPower(0.06);
        }, m_stinger));

    //TEST ELEVATOR UP Position 
    // new JoystickButton(m_driverController, Button.kA.value)
    //    .whileTrue(new RunCommand(() ->{ 
    //    m_elevator.elevatorPIDControl(5);
    //    System.out.println( m_elevator.encoderGetValue());
    //  }, m_stinger));
     //TEST ELEVATOR Down Position 
     new JoystickButton(m_driverController, Button.kY.value)
       .onTrue(new RunCommand(() ->{ 
       m_elevator.elevatorPIDControl(1);
       System.out.println( m_elevator.encoderGetValue());
     }, m_stinger));

    new JoystickButton(m_driverController, Button.kA.value)
       .whileTrue(setWristState(1.5, 0.35));

    new JoystickButton(m_driverController, Button.kX.value)
       .whileTrue(setWristState(22.1, 0.05));
        
  

    //resets the gyro 
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
    //L4
    new JoystickButton(m_operatorController, Button.kX.value)
    .whileTrue(setState(Constants.ElevatorConstants.elevatorL4Position, Constants.PivotConstants.intakeL4Position, 0.93)); 
    //L3
    new JoystickButton(m_operatorController, Button.kY.value)
    .whileTrue(setState(Constants.ElevatorConstants.elevatorL3Position, Constants.PivotConstants.intakeL3Position, 0.77)); 
    //L2
    new JoystickButton(m_operatorController, Button.kB.value)
    .whileTrue(setState(Constants.ElevatorConstants.elevatorL2Position, Constants.PivotConstants.intakeL2Position, 0.69)); 
    //Intake
    new JoystickButton(m_operatorController, Button.kA.value)
    .whileTrue(setState(Constants.ElevatorConstants.elevatorIntakePosition, Constants.PivotConstants.intakePosition, 0.41)); 
  
    //Intaking Coral/Outtaking Algae
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
    .whileTrue(new RunCommand(()-> m_outtake.setIntakePower(0.5,0.8), m_outtake));
    
    //new JoystickButton(m_operatorController, Button.kRightBumper.value)
    //.whileTrue(new OuttakeCommand(m_outtake));
    
    //old outtake coral/intake algae 
    new JoystickButton(m_driverController, Button.kRightBumper.value)
    .whileTrue(new RunCommand(()-> m_outtake.setIntakePower(-0.5, -0.8), m_outtake));
  
    
    /*
    auto align to left side 
    new JoystickButton(m_driverController, Button.kA.value)
    .whileTrue(new AlignToReefTagRelative(false, m_robotDrive));

    auto align to right side 
    new JoystickButton(m_driverController, Button.kB.value)
    .whileTrue(new AlignToReefTagRelative(true, m_robotDrive));

    */

   //barge shot 
    dpadUp.whileTrue(bargeShot(-0.09));
  
    //old algae intakes 
    dpadLeft.whileTrue(setState(13, 0.45, 0.81)); //Algae Low Reef Intake
    dpadDown.whileTrue(setState(0.1, 0.5, 0.61)); // Algae Low Intake
    dpadRight.whileTrue(setState(25, 0.45, 0.87)); //Algae High Reef outtake
  
   //resets elevator encoder 
   new JoystickButton(m_driverController, Button.kStart.value)
   .whileTrue(new RunCommand(()-> {
        if(m_elevator.encoderGetValue() < 5){
          m_elevator.resetEncoder(); 
          System.out.println("LIMIT SET RESESTTIONG");
        }
      }, m_elevator));

   //when limit switch activated, reset the elevator encoder or print there is a stuck coral 
   limitSwitchActivation.whileTrue(
    new RunCommand(() -> {
      if (m_elevator.encoderGetValue() < 2) {
          m_elevator.resetEncoder();
          System.out.println("LIMIT RESETING");
       } 
       else {
           m_elevator.stuckCoral();
       }
    }, m_elevator));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {    
    return straightPath;
  }

  public Command setState(double elevatorPos, double intakePos, double colorLED){
    return Commands.parallel(           
    new RunCommand(() -> m_elevator.elevatorPIDControl(elevatorPos), m_elevator),
    new RunCommand(() -> m_stinger.pivotPIDControl(intakePos), m_stinger),
    //new RunCommand(() -> m_stinger.wristPivotPIDControl(wristIntakePos), m_stinger),// move this entire setstate into commands 
    new RunCommand(() -> m_LED.setPattern(colorLED), m_LED));
  }

  public Command setWristState(double elevatorPos, double wristIntakePos){
    return Commands.parallel(           
    new RunCommand(() -> m_elevator.elevatorPIDControl(elevatorPos), m_elevator),
    new RunCommand(() -> m_stinger.wristPivotPIDControl(wristIntakePos), m_stinger)
    );// move this entire setstate into commands 
  }

  // //SEQUENTIALLY RUNS THE COMMANDS 
  // public Command setSequentialState(double elevatorPos, double intakePos, double colorLED, double wristIntakePos){
  //   return Commands.sequence(           
  //   new RunCommand(() -> m_elevator.elevatorPIDControl(elevatorPos), m_elevator),
  //   new RunCommand(() -> m_stinger.pivotPIDControl(intakePos), m_stinger),
  //   new RunCommand(() -> m_LED.setPattern(colorLED), m_LED));   
  //   new RunCommand(() -> m_stinger.wristPivotPIDControl(wristIntakePos), m_stinger),// move this entire setstate into commands 
  // }


  
  
  
  
  
  
  
  
  
  
  
  public Command bargeShot(double colorLED) {
    return Commands.parallel(
        // Move the elevator to 32.3

        new RunCommand(() -> m_elevator.elevatorPIDControl(30), m_elevator),
        new RunCommand(() -> m_LED.setPattern(colorLED), m_LED),


        // Control the pivot: Start at 0.3, then move to 1 when the elevator is above 30

        new RunCommand(() -> {
            if (m_elevator.encoderGetValue() > 27) {
                m_stinger.pivotPIDControl(0.3);
            } 
            else {
                m_stinger.pivotPIDControl(0.1);
            }
        }, m_stinger),
        new RunCommand(()->{
          if (m_stinger.encoderGetValue() < .38 && m_elevator.encoderGetValue() > 30 ){
            m_outtake.setIntakePower(1,1);
          } else {
            m_outtake.setIntakePower(-0.3,-0.3);
          }
        },  m_outtake)

    );
}


}