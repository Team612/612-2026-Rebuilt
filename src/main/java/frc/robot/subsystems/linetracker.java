package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class linetracker extends TimedRobot {

    private final AnalogInput leftSensor = new AnalogInput(0);
    private final AnalogInput centerSensor = new AnalogInput(1);
    private final AnalogInput rightSensor = new AnalogInput(2);

    @Override
    public void teleopPeriodic() {

        int leftVal = leftSensor.getValue();
        int centerVal = centerSensor.getValue();
        int rightVal = rightSensor.getValue();
        SmartDashboard.putNumber("Left Sensor", leftVal);
        SmartDashboard.putNumber("Center Sensor", centerVal);
        SmartDashboard.putNumber("Right Sensor", rightVal);
    }
}
