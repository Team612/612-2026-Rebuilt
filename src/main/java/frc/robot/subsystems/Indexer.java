package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Indexer subsystem for moving balls from the hopper toward the shooter.
*/
public class Indexer extends SubsystemBase {

  private final PWMVictorSPX indexerMotor;
  private final DigitalInput indexerSensor;

  public Indexer() {
    indexerMotor = new PWMVictorSPX(Constants.IndexerConstants.indexerMotorChannel);
    indexerSensor = new DigitalInput(Constants.IndexerConstants.indexerSensorDIO);
  }

  /** Set indexer motor speed (-1.0..1.0). */
  public void setSpeed(double speed) {
    indexerMotor.set(speed);
  }

  /** Run indexer forward using default configured speed. */
  public void runForward() {
    setSpeed(Constants.IndexerConstants.indexerForwardSpeed);
  }

  /** Run indexer in reverse using default configured speed. */
  public void runReverse() {
    setSpeed(Constants.IndexerConstants.indexerReverseSpeed);
  }

  /** Stop the indexer motor. */
  public void stop() {
    setSpeed(0);
  }

  /**
   * Returns true when a ball is detected in the indexer staging area.
   * This method assumes the sensor returns false when the beam is broken. Invert if needed pls
   */
  public boolean isBallPresent() {
    return !indexerSensor.get();
  }

  @Override
  public void periodic() {
    // telemetry or periodic state checks could go here
  }
}
