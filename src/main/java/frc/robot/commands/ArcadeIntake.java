package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;

public class ArcadeIntake extends Command {

  private Intake m_intake;
  private CommandXboxController controller;

  public ArcadeIntake(Intake m_intake, CommandXboxController controller){
    this.m_intake = m_intake;
    this.controller = controller;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_intake.setMotor(controller.getLeftY());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
