package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class irsensor extends SubsystemBase {

  
  private static I2C.Port i2cPort = I2C.Port.kOnboard;
    
      private final boolean m_inverted;
    
      public irsensor() {
        this(i2cPort, false);
    }
   
    public irsensor(I2C.Port channel, boolean inverted) {
      i2cPort = channel;
    m_inverted = inverted;
  }

  
  public boolean getRaw() {
    return m_inverted;
  }
  public boolean isDetected() {
    boolean raw = getRaw();
    return m_inverted ? !raw : raw;
  }

  @Override
  public void periodic() {
    System.out.printf("IR digital: %s%n", isDetected() ? "DETECTED" : "CLEAR");
  }
}
