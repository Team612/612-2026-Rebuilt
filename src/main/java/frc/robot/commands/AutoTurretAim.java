package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class AutoTurretAim extends Command {

  private Shooter m_shooter;
  
  public AutoTurretAim(Shooter m_shooter) {
    this.m_shooter = m_shooter;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
