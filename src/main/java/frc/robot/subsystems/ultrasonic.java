// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ultrasonic extends SubsystemBase {
  /** Creates a new ultrasonic. */
  private Ultrasonic m_sensor;
  public ultrasonic() {
    m_sensor = new Ultrasonic(0, 1);
  }
  public double getDistance() throws IOException {
    if (!m_sensor.isEnabled()) m_sensor.setEnabled(true);
    if (!m_sensor.isRangeValid()) return 0.0;
    return m_sensor.getRangeMM()*1000;
  }
  public double getDistanceInch() throws IOException {
    if (!m_sensor.isEnabled()) m_sensor.setEnabled(true);
    if (!m_sensor.isRangeValid()) return 0.0;
    return m_sensor.getRangeInches();
  }
  @Override
  public void periodic() {
    try {
      System.out.printf("The distance, in meters is: %.15f\n", getDistance());
    } catch (IOException e) {
      System.out.println(e);
    }
  }
}
