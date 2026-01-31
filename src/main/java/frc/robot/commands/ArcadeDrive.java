package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.TankDrive;


public class ArcadeDrive extends Command {

  private final TankDrive m_drivetrain;
  private final DoubleSupplier forward;
  private final DoubleSupplier turn;

  public ArcadeDrive(TankDrive m_drivetrain, DoubleSupplier forward, DoubleSupplier turn) {
    this.m_drivetrain = m_drivetrain;
    this.forward = forward;
    this.turn = turn;
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double forw = forward.getAsDouble();
    double tur = turn.getAsDouble();

    if (Math.abs(forw) < DriveConstants.DEADBAND)
      forw = 0;
    if (Math.abs(tur) < DriveConstants.DEADBAND)
      tur = 0;
    m_drivetrain.drive(forw, tur);
  }


  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0,0);
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}