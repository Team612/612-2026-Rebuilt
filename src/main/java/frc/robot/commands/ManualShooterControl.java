package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;

public class ManualShooterControl extends Command {

  private Shooter m_shooter;
  private CommandXboxController controller;

  public ManualShooterControl(Shooter m_shooter, CommandXboxController controller) {
    this.m_shooter = m_shooter;
    this.controller = controller;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Reverse - Counter-CW
    // Forward (Green) - CW
    if (controller.getHID().getAButton()){
      m_shooter.setShooterMotor(-0.5);
    }
    else
      m_shooter.setShooterMotor(0.0);

    m_shooter.setTurretMotor(controller.getRightX()*0.1);
    m_shooter.setTiltMotor(controller.getLeftY());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}