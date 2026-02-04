package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj2.command.Command;

public class L1 extends Command {
  Climb m_climb;
  double rotation;

  public L1(Climb m_climb) {
    this.m_climb = m_climb;
    addRequirements(m_climb);
  }

  @Override
  public void initialize() {
    m_climb.SetMotor(0.5);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_climb.SetMotor(0);
  }

  @Override
  public boolean isFinished() {
    if(m_climb.getMotorPosition() >= Constants.ClimbConstants.l1Distance) {
      return true;
    }
    return false;
  }
}
