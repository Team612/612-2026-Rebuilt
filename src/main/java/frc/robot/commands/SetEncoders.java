package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

public class SetEncoders extends InstantCommand {
  private Swerve m_swerve;
  private Pose2d pos;

  public SetEncoders(Swerve m_swerve, Pose2d pos) {
    this.m_swerve = m_swerve;
    this.pos = pos;
    addRequirements(m_swerve);
  }

  @Override
  public void initialize() {
    m_swerve.setPose(pos);
  }
}
