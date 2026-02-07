package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class irsensor extends SubsystemBase {
  private final DigitalInput m_sensor;
  private final boolean m_inverted;

  public irsensor() {
    this(0, false);
  }
 
  public irsensor(int channel, boolean inverted) {
    m_sensor = new DigitalInput(channel);
    m_inverted = inverted;
  }

  
  public boolean getRaw() {
    return m_sensor.get();
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
