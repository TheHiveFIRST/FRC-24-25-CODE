package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    // Variables
    public Spark m_blinkin;
    // constructors
    public LEDSubsystem(){
        m_blinkin = new Spark(2);
    }
    //methods
    public void setPattern(double setPattern){
        m_blinkin.set(setPattern);
    }
    
}
