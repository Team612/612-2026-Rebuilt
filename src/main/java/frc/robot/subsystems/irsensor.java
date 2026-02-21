package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class irsensor extends SubsystemBase implements AutoCloseable{

  private AnalogInput m_sensor;
  private final double a, b, min, max;
  private final long LSB_Weight;
  private final int offset;
  private boolean in_range;
  private int counter = -1;
  private boolean fuel = false;
  private boolean available = true;

  
  /**
  * @author: Achyut Dipukumard
  * @team: 612<br><br>
  * This code is derived from edu.wpi.first.wpilibj.SharpIR class.<br><br>
  * This code will not work with Simulation, only physical objects will work.<br><br>
  * This code will return about 2 cm error, more accurate than the actual SharpIR library.<br><br>
  * The recommended range for the sensor is 20 cm to 100 cm, but it can theoretically measure up to 150 cm.<br><br>
  * The code should also be faster than the SharpIR library as the voltage usage here is faster than the default WPILIB voltage calculations.<br><br>
  * The only way to be faster would be to utilize JNI optimizations, preferably with JNA. Talk to me if the code does need to be faster.<br><br>
  */
  public static irsensor M2Y0A02(int channel) {
    return new irsensor(channel, 0.5922, -1.08, 0.20, 1.500);
  }

  public static irsensor M2Y0A21(int channel) {
    return new irsensor(channel, 26.449, -1.226, 10.0, 80.0);
  }

  public static irsensor M2Y0A41(int channel) {
    return new irsensor(channel, 12.354, -1.07, 4.0, 30.0);
  }

  public static irsensor M2Y0A51(int channel) {
    return new irsensor(channel, 5.2819, -1.161, 2.0, 15.0);
  }
  private irsensor(int channel, double a, double b, double min, double max) {
    m_sensor = new AnalogInput(channel);
    LSB_Weight = m_sensor.getLSBWeight();
    offset = m_sensor.getOffset();
    this.a = a;
    this.b = b;
    this.min = min;
    this.max = max;
    SendableRegistry.addLW(this, "IR Sensor", channel);
  }
  @Override
  public void close() {
    SendableRegistry.remove(this);
    m_sensor.close();
    m_sensor = null;
  }
  public int getChannel() {
    return m_sensor.getChannel();
  }
  public double getDistanceMeters() {
    var v = Math.max(getRawVoltage(), 0.00001);
    double distance = a*Math.pow(v, b);
    if (distance > max) {
      in_range = false;
      return max;
    } else if (distance < min) {
      in_range = false;
      return min;
    }
    in_range = true;
    return distance;
  }
  public boolean objectExists() {
    return in_range;
  }
  public double getRawVoltage() { // for optimization
    // return m_sensor.getVoltage();
    // for direct calculation use the following:
    return (LSB_Weight * m_sensor.getValue() - offset) * 1.0e-9;
  }
  public boolean fuelExists() {
    fuel = getRawVoltage() < Constants.SensorConstants.kVoltagePeak;
    return fuel;
  }
  public void countBalls() {
    if(available && fuelExists()) {
      available = false;
      counter++;
    }
    else if(!available && !fuelExists()) {
      available = true;
    }
  }
  public int getCount() {
    return counter;
  }
  @Override
  public void initSendable(SendableBuilder b) {
    b.setSmartDashboardType("Ultrasonic");
    b.addDoubleProperty("Value", this::getDistanceMeters, null);
  }

  @Override
  public void periodic() {
    System.out.printf("IR digital: %s%n", isDetected() ? "DETECTED" : "CLEAR");
  }
}
