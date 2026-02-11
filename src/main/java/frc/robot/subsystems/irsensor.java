package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Micro;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SharpIR;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class irsensor extends SubsystemBase {
  DigitalInput irSensor = new DigitalInput(0);
  SharpIR distanceSensor = SharpIR.GP2Y0A21YK0F(0);
  private final I2C.Port i2cPort;
  private final boolean m_inverted;  
    public irsensor() {
      i2cPort = I2C.Port.kOnboard;
      m_inverted = false;
    }
  public irsensor(I2C.Port channel, boolean inverted) {
    i2cPort = channel;
    m_inverted = inverted;
  }
  public boolean getRaw() {
    return m_inverted;
  }
  public boolean isDetected() {
    return m_inverted;
  }
  public double getDistance() {
    return distanceSensor.getRangeCM()*100;
  }
  public boolean isThere() {
    return irSensor.get();
  }
  @Override
  public void periodic() {
    // System.out.printf("IR digital: %s%n", isDetected() ? "DETECTED" : "CLEAR");
    // System.out.println(isThere() + " - " + getDistance());
  }
}
