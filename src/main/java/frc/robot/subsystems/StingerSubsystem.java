package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


//import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

public class StingerSubsystem extends SubsystemBase {
    // Variables
    private SparkMax m_pivotMotor;
    private SparkMaxConfig pivotConfig;
    private AbsoluteEncoder m_absoluteEncoder;
    private PIDController m_pivotPID;
    public double pivotOutput = 0;
   
    private SparkMax m_wristPivotMotor; 
    private SparkMaxConfig wristPivotConfig; 
    //private SoftLimitConfig softLimitConfig; 
    private AbsoluteEncoder m_wristAbsoluteEncoder;
    private PIDController m_wristPivotPID;
    public double wristPivotOutput = 0;


    // Constructors
    public StingerSubsystem() {
        m_pivotMotor = new SparkMax(Constants.PivotConstants.kPivotMotorId, MotorType.kBrushless);
        pivotConfig = new SparkMaxConfig();
        pivotConfig.idleMode(IdleMode.kBrake);
        pivotConfig.inverted(true);
        pivotConfig.smartCurrentLimit(Constants.PivotConstants.pivotCurrentLimit); 
        m_pivotMotor.configure(pivotConfig, null, null);
        
        m_absoluteEncoder =  m_pivotMotor.getAbsoluteEncoder();
        m_pivotPID = new PIDController(Constants.PivotConstants.pivotKP,Constants.PivotConstants.pivotKI, Constants.PivotConstants.pivotKD);
    
        m_wristPivotMotor = new SparkMax(Constants.PivotConstants.kWristPivotMotorId, MotorType.kBrushless);
        wristPivotConfig = new SparkMaxConfig();
        wristPivotConfig.idleMode(IdleMode.kBrake);
        wristPivotConfig.inverted(true);
        wristPivotConfig.smartCurrentLimit(Constants.PivotConstants.wristPivotCurrentLimit);
        //softLimitConfig = new SoftLimitConfig();
        //softLimitConfig.forwardSoftLimit(0.2);
        //softLimitConfig.forwardSoftLimitEnabled(true);
        //softLimitConfig.reverseSoftLimit(0.2);
        //softLimitConfig.reverseSoftLimitEnabled(true);
        //wristPivotConfig.apply(softLimitConfig);
        m_wristPivotMotor.configure(wristPivotConfig, null, null);
        
        m_wristAbsoluteEncoder =  m_wristPivotMotor.getAbsoluteEncoder();
        
        m_wristPivotPID = new PIDController(Constants.PivotConstants.wristPivotKP,Constants.PivotConstants.wristPivotKI, Constants.PivotConstants.wristPivotKD);
        

    }


    public void setPivotPower(double pivotPower){
        m_wristPivotMotor.set(pivotPower);

        //System.out.println(pivotPower);
    }

    public void pivotPIDControl(double targetAngle) {
        pivotOutput = m_pivotPID.calculate(m_absoluteEncoder.getPosition(), targetAngle);
        m_pivotMotor.set(pivotOutput);  
        System.out.println("pivot pid ran, pivot output was" + pivotOutput);
    }

    public void wristPivotPIDControl(double targetWristAngle) {
        wristPivotOutput = m_wristPivotPID.calculate(m_wristAbsoluteEncoder.getPosition(), targetWristAngle);
        m_wristPivotMotor.set(wristPivotOutput);

        System.out.println("pivot pid ran, wrist output was" + wristPivotOutput);
    }

     public double encoderGetValue(){
        return m_absoluteEncoder.getPosition();
    }

    public double getCurrentWristAngle(){
        return m_wristAbsoluteEncoder.getPosition();
    }
}

