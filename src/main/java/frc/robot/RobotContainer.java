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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
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


import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

//import java.util.HashMap;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final LEDSubsystem m_LED = new LEDSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final StingerSubsystem m_stinger = new StingerSubsystem();
  private final OuttakeSubsystem m_outtake = new OuttakeSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);


  private PathPlannerAuto ishanaPath = new PathPlannerAuto("Blue Side");

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */  
  public RobotContainer() {    // Configure the button bindings

    configureButtonBindings();
    m_elevator.setDefaultCommand(new RunCommand(()-> m_elevator.elevatorPIDControl(0), m_elevator));
    m_stinger.setDefaultCommand(new RunCommand(()-> m_stinger.pivotPIDControl(0.3), m_stinger));
    m_outtake.setDefaultCommand(new RunCommand(()-> m_outtake.setIntakePower(0), m_outtake));
    m_LED.setDefaultCommand(new RunCommand(()-> m_LED.setPattern(-0.99), m_LED));
    

  
    
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

   /* created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    POVButton dpadUp = new POVButton(m_operatorController, 0);
    POVButton dpadRight = new POVButton(m_operatorController, 90);
    POVButton dpadDown = new POVButton(m_operatorController, 180);
    POVButton dpadLeft = new POVButton(m_operatorController, 270);
    new JoystickButton(m_driverController, Button.kX.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, Button.kRightBumper.value)
            .whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
    
    new JoystickButton(m_operatorController, Button.kX.value)
    .whileTrue(setState(32.3, .52, 0.91)); 
    //L4
    new JoystickButton(m_operatorController, Button.kY.value)
    .whileTrue(setState(18.6, .5, 0.81)); 
    //L3
    new JoystickButton(m_operatorController, Button.kB.value)
    .whileTrue(setState(9.1, 0.5, 0.73)); 
    //L2
    new JoystickButton(m_operatorController, Button.kA.value)
    .whileTrue(setState(0.1, 0.5, 0.57)); 
    //Ground Intake
  
  
    new JoystickButton(m_operatorController, Button.kLeftBumper.value)
    .whileTrue(new RunCommand(()-> m_outtake.setIntakePower(0.3), m_outtake));
    //Intaking Coral/Outtaking Algae
    new JoystickButton(m_operatorController, Button.kRightBumper.value)
    .whileTrue(new RunCommand(()-> m_outtake.setIntakePower(-0.3), m_outtake));
    //Outtaking Coral/Intaking Algae

   dpadDown.whileTrue(bargeShot());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {    
    return ishanaPath;
  }
  public Command setState(double elevatorPos, double intakePos, double colorLED){
    return Commands.parallel(           
    new RunCommand(() -> m_elevator.elevatorPIDControl(elevatorPos), m_elevator),
    new RunCommand(() -> m_stinger.pivotPIDControl(intakePos), m_stinger),
    new RunCommand(() -> m_LED.setPattern(colorLED), m_LED));
  }

  public Command bargeShot(){
    return Commands.sequence(
      new RunCommand(()->m_stinger.pivotPIDControl(.5), m_stinger),

      new RunCommand(()-> m_elevator.elevatorPIDControl(32.3), m_elevator),
      waitUntil(()->m_elevator.encoderGetValue() > 31),
      new RunCommand(()->m_stinger.pivotPIDControl(.1), m_stinger),
      waitUntil(()->m_stinger.encoderGetValue() > 0.6),
      new RunCommand(()-> m_outtake.setIntakePower(1), m_outtake)

    );
  }

}