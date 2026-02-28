package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TankDrive;

public class AutonomousRoutine extends Command {

  private final TankDrive m_tankDrive;
  private int timer = 0;

  public AutonomousRoutine(TankDrive m_tankDrive) {
    this.m_tankDrive = m_tankDrive;
    addRequirements(m_tankDrive);
  }

  @Override
  public void initialize() {
    m_tankDrive.arcadeDrive(0.1,0);
  }

  @Override
  public void execute() {
    timer += 1;
  }

  @Override
  public void end(boolean interrupted) {
    m_tankDrive.arcadeDrive(0.1,0);
  }

  @Override
  public boolean isFinished() {
    return (timer > 60);
  }
}
