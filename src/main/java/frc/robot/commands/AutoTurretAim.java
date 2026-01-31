package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Shooter;

public class AutoTurretAim extends Command {

  private Shooter m_shooter;

  public AutoTurretAim(Shooter m_shooter) {
    this.m_shooter = m_shooter;
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {


    m_shooter.setTurretPos(Math.PI/6);


  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
