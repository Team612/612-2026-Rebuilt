package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class ManualShooterControl extends Command {

  private Shooter m_shooter;
  private DoubleSupplier turretSupplier;
  private DoubleSupplier tiltSupplier;
  private BooleanSupplier shootSupplier;

  public ManualShooterControl(Shooter m_shooter, DoubleSupplier turretSupplier, DoubleSupplier tiltSupplier, BooleanSupplier shootSupplier){
    this.m_shooter = m_shooter;
    this.turretSupplier = turretSupplier;
    this.tiltSupplier = tiltSupplier;
    this.shootSupplier = shootSupplier;
    addRequirements(m_shooter);
    SmartDashboard.putNumber("Shooter RPM", 3000);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (shootSupplier.getAsBoolean())
      m_shooter.setShooterRPM(SmartDashboard.getNumber("Shooter RPM", 3000));
    else
      m_shooter.setShooterRPM(0);

    m_shooter.setTurretMotor(turretSupplier.getAsDouble() * ShooterConstants.maxTurretSpeed);
    m_shooter.setTiltMotor(tiltSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}