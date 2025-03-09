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

public class StingerSubsystem extends SubsystemBase {
    // Variables
    public SparkMax m_intakeMotor; 
    private SparkMax m_pivotMotor;
    private SparkMaxConfig pivotConfig;
    private AbsoluteEncoder m_absoluteEncoder;
    private PIDController m_pivotPID;
    public double pivotOutput = 0;

    // Constructors
    public StingerSubsystem() {
        m_intakeMotor = new SparkMax(Constants.PivotConstants.kPintakeMotorId, MotorType.kBrushless);
        m_pivotMotor = new SparkMax(Constants.PivotConstants.kPivotMotorId, MotorType.kBrushless);
        pivotConfig = new SparkMaxConfig();
        pivotConfig.idleMode(IdleMode.kCoast);
        pivotConfig.inverted(true);
        m_pivotMotor.configure(pivotConfig, null, null);
        
        m_absoluteEncoder =  m_pivotMotor.getAbsoluteEncoder();
        m_pivotPID = new PIDController(Constants.PivotConstants.pivotKP,Constants.PivotConstants.pivotKI, Constants.PivotConstants.pivotKD);
    }

    // Methods
    public void setIntakePower(double intakePower){
        m_intakeMotor.set(intakePower);
    }

    public void setPivotPower(double pivotPower){
        m_pivotMotor.set(pivotPower);
    }

    public void pivotPIDControl(double targetAngle) {
        // pivotOutput = m_pivotPID.calculate(m_absoluteEncoder.getPosition(), targetAngle);
        // m_pivotMotor.set(pivotOutput);  

        System.out.println("pivot pid ran, target angle was" + targetAngle);
    }

     public void encoderGetValue(){
        System.out.println("Pivot Position" + m_absoluteEncoder.getPosition());
       }

}