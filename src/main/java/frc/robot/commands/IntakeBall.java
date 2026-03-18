package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeBall extends Command {

  private final Intake m_intake;
 
  public IntakeBall(Intake m_intake) {
    this.m_intake = m_intake;
    addRequirements(m_intake);
    SmartDashboard.putNumber("Intake Ball 1", IntakeConstants.upperIntakeSpeed);
    SmartDashboard.putNumber("Intake Ball 2", IntakeConstants.lowerIntakeSpeed);
    SmartDashboard.putNumber("AgitatorIntake", IntakeConstants.jammerIntakeSpeed);
  }

  @Override
  public void initialize() {
    m_intake.setUpperIntakeSpeed(SmartDashboard.getNumber("Intake Ball 1",0.35));
    m_intake.setLowerIntakeSpeed(SmartDashboard.getNumber("Intake Ball 2",-0.05));
    m_intake.setJammerSpeed(SmartDashboard.getNumber("AgitatorIntake", -0.2));
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
