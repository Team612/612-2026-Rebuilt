package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TankDrive;

public class AutoTurretAim extends Command {

  private Shooter m_shooter;
  private TankDrive m_tankDrive;

  private boolean red;

  private double hubXpos;

  public AutoTurretAim(Shooter m_shooter, TankDrive m_tankDrive) {
    this.m_shooter = m_shooter;
    this.m_tankDrive = m_tankDrive;
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
      Pose2d robotPos = m_tankDrive.getPose();
  
      double xdiff = hubXpos - robotPos.getX();
      double ydiff = OperatorConstants.hubYPos - robotPos.getY();

      double desiredTheta = Math.atan2(ydiff,xdiff);
      desiredTheta -= robotPos.getRotation().getRadians();

      desiredTheta = Math.IEEEremainder(desiredTheta,2*Math.PI);

      m_shooter.setTurretPos(desiredTheta);
      m_shooter.setTiltPos(m_shooter.getRegressionModelTilt(Math.sqrt(xdiff*xdiff+ydiff*ydiff)));
    }
    else {
      double desiredTheta = m_shooter.getCurrentTurretAngle() + m_shooter.calculateShootingAnglesWithOfficialOffset()[0];

      desiredTheta = Math.IEEEremainder(desiredTheta,2*Math.PI);

      m_shooter.setTurretPos(desiredTheta);
      m_shooter.setTiltPos(m_shooter.getRegressionModelTilt(m_shooter.calculateShootingAnglesWithOfficialOffset()[1]));
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}