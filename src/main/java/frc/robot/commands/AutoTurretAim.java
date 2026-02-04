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
    

    if(m_shooter.shooterHasTag()){
        double offset =  m_shooter.calculateShootingAnglesWithOfficialOffset()[0];
        double current = m_shooter.getCurrentTurretAngle();
        m_shooter.setTurretPos(current += offset);
    }

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
