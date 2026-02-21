// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Shooter;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FeedAndShoot extends Command {
  /** Creates a new FeedAndShoot. */
  private final Transfer m_transfer;
  private final Shooter m_shooter;
  private final Timer m_timer = new Timer();

  public FeedAndShoot(Transfer transfer, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_transfer = transfer;
    m_shooter = shooter;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_transfer.setFeed(0.6);
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_timer.hasElapsed(0.5)) {
      m_shooter.setShooterMotor(0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setShooterMotor(0);
    m_transfer.setFeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
