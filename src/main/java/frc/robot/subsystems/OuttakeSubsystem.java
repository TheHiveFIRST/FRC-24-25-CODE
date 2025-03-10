package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

public class OuttakeSubsystem extends SubsystemBase {
    // Variables
    public SparkMax m_intakeMotor; 


    // Constructors
    public OuttakeSubsystem() {
        m_intakeMotor = new SparkMax(Constants.PivotConstants.kPintakeMotorId, MotorType.kBrushless);
    }

    // Methods
    public void setIntakePower(double intakePower){
        m_intakeMotor.set(intakePower);
    }

}