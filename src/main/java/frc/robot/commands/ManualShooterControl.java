package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ShooterConstants;
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
    m_shooter.setShooterMotor(controller.getRightTriggerAxis() - controller.getLeftTriggerAxis());

    double y = controller.getLeftY();

    if (Math.abs(y) < ShooterConstants.DEADBAND) y = 0;

    double xComponent = controller.getRightX();
    double yComponent = controller.getRightY();

    double desiredRadians = Math.atan2(yComponent,xComponent);

    m_shooter.setTurretPos(desiredRadians);
    m_shooter.setTiltMotor(y);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}