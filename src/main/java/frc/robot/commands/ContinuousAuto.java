package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Shooter;

public class ContinuousAuto extends Command {

  private Shooter m_shooter;
  private boolean forward = false;

  private double hubXpos;

  // this is a stand in for the drivetrain replace when done
  public Pose2d getPose(){
    return new Pose2d(0,4.034536,new Rotation2d());
  }

  public ContinuousAuto(Shooter m_shooter) {
    this.m_shooter = m_shooter;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    

  }

  @Override
  public void execute() {
    if(m_shooter.GetLeftTurretLimits()){
        forward = true;
    }
    if(m_shooter.GetRightTurretLimits()){
        forward = false;
    }

    if (forward){
        m_shooter.setTurretMotor(0.2);
    }
    else{
        m_shooter.setTurretMotor(-0.2);
    }

    if (m_shooter.hasTag()){
      double desiredTheta = m_shooter.getCurrentTurretAngle() + m_shooter.calculateShootingAnglesWithOfficialOffset()[0];

      Math.IEEEremainder(desiredTheta,2*Math.PI);

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