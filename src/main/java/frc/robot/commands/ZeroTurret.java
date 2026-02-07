package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

public class ZeroTurret extends InstantCommand {

  private Shooter m_shooter;

  public ZeroTurret(Shooter m_shooter) {
    this.m_shooter = m_shooter;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    m_shooter.setTurretEncoderPosition(0);
    m_shooter.setTiltPos(0);
  }
}
