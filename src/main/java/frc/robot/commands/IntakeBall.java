package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeBall extends Command {

  private final Intake m_intake;
 
  public IntakeBall(Intake m_intake) {
    this.m_intake = m_intake;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_intake.setUpperIntakeSpeed(IntakeConstants.upperIntakeSpeed);
    m_intake.setLowerIntakeSpeed(IntakeConstants.lowerIntakeSpeed);
    m_intake.setJammerSpeed(IntakeConstants.jammerIntakeSpeed);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_intake.setUpperIntakeSpeed(0);
    m_intake.setLowerIntakeSpeed(0);
    m_intake.setJammerSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
