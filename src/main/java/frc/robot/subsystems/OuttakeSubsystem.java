package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OuttakeSubsystem extends SubsystemBase {
    // Variables
    private SparkMax m_coralIntakeMotor; 
    private SparkMaxConfig coralIntakeMotorConfig; 
    private SparkMax m_algaeIntakeMotor; 
    private SparkMaxConfig algaeIntakeMotorConfig; 



    // Constructors
    public OuttakeSubsystem() {
        m_coralIntakeMotor = new SparkMax(Constants.OuttakeConstants.coralIntakeMotorId, MotorType.kBrushless);
        coralIntakeMotorConfig.idleMode(IdleMode.kBrake);
        m_coralIntakeMotor.configure(coralIntakeMotorConfig, null, null);
        m_algaeIntakeMotor = new SparkMax(Constants.OuttakeConstants.algaeIntakeMotorId, MotorType.kBrushless);
        algaeIntakeMotorConfig.idleMode(IdleMode.kBrake); 
        m_algaeIntakeMotor.configure(algaeIntakeMotorConfig, null, null);

    }

    // Methods
    public void setIntakePower(double coralIntakePower, double algaeIntakePower) {
        m_coralIntakeMotor.set(coralIntakePower);
        m_algaeIntakeMotor.set(algaeIntakePower);
    }

}