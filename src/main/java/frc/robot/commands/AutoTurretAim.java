package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Shooter;

public class AutoTurretAim extends Command {

  private Shooter m_shooter;
  private boolean red;

  private double hubXpos;

  // this is a stand in for the drivetrain replace when done
  public Pose2d getPose(){
    return new Pose2d();
  }

  public AutoTurretAim(Shooter m_shooter) {
    this.m_shooter = m_shooter;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
        red = true;
    }
    if (red)
      hubXpos = OperatorConstants.redHubXPos;
    else 
      hubXpos = OperatorConstants.blueHubXPos;

  }

  @Override
  public void execute() {
    if ((m_shooter.calculateShootingAnglesWithOfficialOffset()[0] == -1) && (m_shooter.calculateShootingAnglesWithOfficialOffset()[1] == -1)){
      Pose2d robotPos = getPose();
  
      double xdiff = hubXpos - robotPos.getX();
      double ydiff = OperatorConstants.hubYPos - robotPos.getY();

      double desiredTheta = Math.atan2(ydiff,xdiff);
      desiredTheta -= robotPos.getRotation().getRadians();

      Math.IEEEremainder(desiredTheta,2*Math.PI);

      m_shooter.setTurretPos(desiredTheta);
    }
    else {
      double desiredTheta = m_shooter.getCurrentTurretAngle() + m_shooter.calculateShootingAnglesWithOfficialOffset()[0];

      Math.IEEEremainder(desiredTheta,2*Math.PI);

      m_shooter.setTurretPos(desiredTheta);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
