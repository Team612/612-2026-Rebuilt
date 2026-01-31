package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
  private final PWMVictorSPX hopperMotorA;
  private final PWMVictorSPX hopperMotorB;
  private final MotorControllerGroup hopperMotors;
  private final DigitalInput entranceSensor;

  public Hopper() {
    hopperMotorA = new PWMVictorSPX(Constants.HopperConstants.hopperMotorAChannel);
    hopperMotorB = new PWMVictorSPX(Constants.HopperConstants.hopperMotorBChannel);
    hopperMotors = new MotorControllerGroup(hopperMotorA, hopperMotorB);

    entranceSensor = new DigitalInput(Constants.HopperConstants.hopperSensorDIO);
  }

  // Set hopper motor speed (-1.0...1.0) for both motors
  public void setSpeed(double speed) {
    hopperMotors.set(speed);
  }

  // Run the hopper forward using the default forward speed 
  public void runForward() {
    setSpeed(Constants.HopperConstants.hopperForwardSpeed);
  }

  // Run the hopper in reverse using the default reverse speed
  public void runReverse() {
    setSpeed(Constants.HopperConstants.hopperReverseSpeed);
  }

  // Stop the hopper motors.
  public void stop() {
    setSpeed(0);
  }

  // Returns true when a ball is detected at the hopper entrance becuz why not
  public boolean isBallDetected() {
    return !entranceSensor.get();
  }

  @Override
  public void periodic() {
    
  }
}
