package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TankDrive;

public class AutonomousRoutineBump extends Command {

  private final TankDrive m_tankDrive;
  private int timer;

  public AutonomousRoutineBump(TankDrive m_tankDrive) {
    this.m_tankDrive = m_tankDrive;
    addRequirements(m_tankDrive);
  }

  @Override
  public void initialize() {
    m_tankDrive.manualSetKrakenMotors(-0.1);
    timer = 0;
  }

  @Override
  public void execute() {
    m_tankDrive.manualSetKrakenMotors(-0.1);
    timer += 1;
  }

  @Override
  public void end(boolean interrupted) {
    m_tankDrive.manualSetKrakenMotors(0);
  }

  @Override
  public boolean isFinished() {
    return (timer > 300);
  }
}
