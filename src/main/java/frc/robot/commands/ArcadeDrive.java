package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.TankDrive;

public class ArcadeDrive extends Command {

  private final TankDrive m_tankDrive;
  private final CommandXboxController controller;

  public ArcadeDrive(TankDrive m_tankDrive, CommandXboxController controller) {
    this.controller = controller;
    this.m_tankDrive = m_tankDrive;
    addRequirements(m_tankDrive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_tankDrive.arcadeDrive(
    -0.5*controller.getLeftY(),
    -0.5*controller.getRightX()
    );
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}