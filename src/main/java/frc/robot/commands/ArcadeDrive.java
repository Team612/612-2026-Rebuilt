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
    double speed = controller.getLeftY() * 0.6;
    double turn = controller.getRightX() * 0.6;

    // if (Math.abs(speed) <= 0.5)
    //   speed = speed*0.6;
    // else if (speed > 0.5)
    //   speed = 1.6 * speed * speed - speed + 0.4;
    // else
    //   speed = -1.6 * speed * speed - speed - 0.4;

    // if (Math.abs(turn) <= 0.5)
    //   turn = turn*0.6;
    // else if (turn > 0.5)
    //   turn = 1.6 * turn * turn - turn + 0.4;
    // else
    //   turn = -1.6 * turn * turn - turn - 0.4;

    m_tankDrive.arcadeDrive(
    -speed,
    -turn
    );
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}