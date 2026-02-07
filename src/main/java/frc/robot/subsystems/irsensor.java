package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class irsensor extends SubsystemBase {
  
  private final DigitalInput m_sensor;

  public irsensor(int port) {
    m_sensor = new DigitalInput(port);
  }

  
  public boolean isObjectDetected() {
    // Invert the reading so that 'true' means "object detected"
    // Adjust this line if your sensor works differently
    return !m_sensor.get();
  }
  
  @Override
  public void periodic() {
    // This method is called once per scheduler run.
    // Use this to continuously update data on the SmartDashboard for debugging.
    SmartDashboard.putBoolean("IR Sensor Detected", isObjectDetected());
  }

  public boolean isClear() {
    return !isObjectDetected();
  }
}
