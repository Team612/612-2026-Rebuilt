// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class irsensor extends SubsystemBase {
  /** Creates a new irsensor. */
  private DigitalInput m_sensor;

  public irsensor(int port) {
    m_sensor = new DigitalInput(port);
  }

  public boolean objectDetected() {
    return m_sensor.get();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
