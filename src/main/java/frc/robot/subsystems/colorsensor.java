
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;

public class colorsensor extends TimedRobot {

  private final I2C.Port i2cPort = I2C.Port.kOnboard;


  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  @Override
public void robotPeriodic() {

    int red = m_colorSensor.getRed();
    int green = m_colorSensor.getGreen();
    int blue = m_colorSensor.getBlue();
    int ir = m_colorSensor.getIR();

    SmartDashboard.putNumber("Red", red);
    SmartDashboard.putNumber("Green", green);
    SmartDashboard.putNumber("Blue", blue);
    SmartDashboard.putNumber("IR", ir);

    int proximity = m_colorSensor.getProximity();
    SmartDashboard.putNumber("Proximity", proximity);
}

}