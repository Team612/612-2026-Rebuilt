package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TankDrive;

public class AutonomousRoutine extends Command {

  private final TankDrive m_tankDrive;

  public AutonomousRoutine(TankDrive m_tankDrive) {
    this.m_tankDrive = m_tankDrive;
    addRequirements(m_tankDrive);
  }

  @Override
  public void initialize() {
    m_tankDrive.drive(new ChassisSpeeds(0.1,0,0));
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_tankDrive.drive(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    // System.out.println(m_tankDrive.getLeftDistanceMeters());

    if (m_tankDrive.getLeftDistanceMeters() > 1) {
      return true;
    }
    return false;
  }
}
