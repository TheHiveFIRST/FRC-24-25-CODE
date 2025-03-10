package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
//variables
private SparkMax m_elevatorMotor;
private SparkMax m_elevatorFollower;
private PIDController m_elevatorPID;
private Encoder m_elevatorEncoder;
private SparkMaxConfig leftConfig;
private SparkMaxConfig rightConfig;
public double output = 0;
SlewRateLimiter lessFast = new SlewRateLimiter(1);
//constructors
    public ElevatorSubsystem(){
        m_elevatorMotor = new SparkMax(Constants.ElevatorConstants.kElevatorMotorCanId, MotorType.kBrushless);
        m_elevatorFollower = new SparkMax(Constants.ElevatorConstants.kElevatorFollowerCanId, MotorType.kBrushless);
        
        m_elevatorEncoder = new Encoder(Constants.ElevatorConstants.KEncoderChannelA, Constants.ElevatorConstants.KEncoderChannelB);
        m_elevatorEncoder.setDistancePerPulse(2*Math.PI*2/4096);
        m_elevatorPID = new PIDController(Constants.ElevatorConstants.KP, Constants.ElevatorConstants.KI, Constants.ElevatorConstants.KD);

        leftConfig = new SparkMaxConfig();
        rightConfig = new SparkMaxConfig();
        leftConfig.idleMode(IdleMode.kCoast);
        rightConfig.idleMode(IdleMode.kCoast);

        leftConfig.inverted(false);
        rightConfig.inverted(true);

        m_elevatorMotor.configure(leftConfig, null, null);
        m_elevatorFollower.configure(rightConfig, null, null);
    }

    // methods

    public void setElevatorPower(double elevatorPower){
        m_elevatorMotor.set(elevatorPower);
        m_elevatorFollower.set(elevatorPower);
        //System.out.println("Encoder Position" + m_elevatorEncoder.getDistance());

    }
    public void elevatorPIDControl(double setPosition){
        output = m_elevatorPID.calculate(m_elevatorEncoder.getDistance(), setPosition);
        m_elevatorMotor.set(output);
        m_elevatorFollower.set(output);
        System.out.println("Elevator pid ran, target angle was" + setPosition);

    }


    public void resetEncoder(){
        m_elevatorEncoder.reset();
    }
    
    public double encoderGetValue(){
    return m_elevatorEncoder.getDistance();
    }
}