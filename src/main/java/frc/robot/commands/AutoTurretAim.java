package frc.robot.commands;

import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
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
    Translation2d cv = Translation2d.kZero;
    if ((m_shooter.calculateShootingAnglesWithOfficialOffset()[0] == -1) && (m_shooter.calculateShootingAnglesWithOfficialOffset()[1] == -1)){
      Pose2d turretPose = m_tankDrive.getPose().plus(ShooterConstants.robotToTurret2d);
      long fTime = System.nanoTime();
      double xdiff = hubXpos - turretPose.getX();
      double ydiff = OperatorConstants.hubYPos - turretPose.getY();
      double dist = Math.sqrt(xdiff*xdiff+ydiff*ydiff);
      double angle = Math.IEEEremainder(m_shooter.getRegressionModelTilt(dist)*ShooterConstants.turretEncoderToRadians, 2*Math.PI);
      long endTime = System.nanoTime();
      turretPose = m_tankDrive.getPose().plus(ShooterConstants.robotToTurret2d);
      cv = new Translation2d(xdiff-hubXpos+turretPose.getX(), ydiff-OperatorConstants.hubYPos+turretPose.getY());
      cv.div((endTime-fTime)*1000000000);
      if (cv != Translation2d.kZero) {
        double cosAngle = Math.cos(angle);
        double nv = Math.sqrt(9.8*(xdiff*xdiff+ydiff*ydiff)/(2*cosAngle*cosAngle*(dist*Math.tan(angle)-OperatorConstants.hubYPos+ShooterConstants.shooterHeight)));
        Translation2d nvv = new Translation2d(nv, new Rotation2d(angle));
        Translation2d fv = new Translation2d(Math.abs(nvv.getX()-cv.getX()), Math.abs(nvv.getY()-cv.getY()));
        double finalAngle = fv.getAngle().getRadians();
        m_shooter.setTiltPos(finalAngle*Math.PI*2/ShooterConstants.turretEncoderToRadians);
      }
      double desiredTheta = Math.atan2(ydiff,xdiff);
      desiredTheta -= turretPose.getRotation().getRadians();

      desiredTheta = Math.IEEEremainder(desiredTheta,2*Math.PI);

      m_shooter.setTurretPos(desiredTheta);
      m_shooter.setTiltPos(m_shooter.getRegressionModelTilt(Math.sqrt(xdiff*xdiff+ydiff*ydiff)));
      SmartDashboard.putNumber("HubDist",Math.sqrt(xdiff*xdiff+ydiff*ydiff));
    }
    else {
      double desiredTheta = m_shooter.getCurrentTurretAngle() + m_shooter.calculateShootingAnglesWithOfficialOffset()[0];

      desiredTheta = Math.IEEEremainder(desiredTheta,2*Math.PI);

      m_shooter.setTurretPos(desiredTheta);
      m_shooter.setTiltPos(m_shooter.getRegressionModelTilt(m_shooter.calculateShootingAnglesWithOfficialOffset()[1]));

      SmartDashboard.putNumber("HubDist",m_shooter.calculateShootingAnglesWithOfficialOffset()[1]);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}