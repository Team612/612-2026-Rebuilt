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
    Pose2d robotPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0)); 
    
    double xLen = true ? OperatorConstants.blueHubXPos - robotPose.getX() : OperatorConstants.redHubPos - robotPose.getX();
    double yLen = OperatorConstants.hubYPos - robotPose.getY();


    double desiredRadians = Math.atan2(yLen, xLen) - robotPose.getRotation().getRadians();

    // m_shooter.setTurretPos(desiredRadians);


  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
