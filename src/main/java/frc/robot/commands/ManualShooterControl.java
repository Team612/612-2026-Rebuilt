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

    double x = controller.getRawAxis(1);
    double y = controller.getRawAxis(0);
    double rad = -Math.atan2(x,y);

    rad -= Math.PI/2;
    if (rad < -Math.PI)
      rad+= 2 * Math.PI;
    
    m_shooter.setTurretPos(rad);

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}