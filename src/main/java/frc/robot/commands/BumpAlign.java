package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants.LightID;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class BumpAlign extends Command {

  private Swerve m_swerve;
  private Vision m_vision;
  private PIDController pid = new PIDController(DriveConstants.bumpAlignKp,DriveConstants.bumpAlignKi,DriveConstants.bumpAlignKd);

  public BumpAlign(Swerve m_swerve, Vision m_vision) {
    this.m_swerve = m_swerve;
    this.m_vision = m_vision;
  }

  @Override
  public void initialize() {
    m_vision.setLightID(LightID.GREEN.id);
  }

  @Override
  public void execute() {
    double curr = m_swerve.getHeading().getDegrees() % 90;

    if (curr < 0){
      curr *= -1;
      curr = 90 - curr;
    }

    m_swerve.setAutoComponent(new ChassisSpeeds(0,0,pid.calculate((curr - 45),0)));
  }

  @Override
  public void end(boolean interrupted) {
    m_vision.setLightID(LightID.OFF.id);
    m_swerve.setAutoComponent(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}