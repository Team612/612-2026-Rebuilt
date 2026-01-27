package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.LightID;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class AprilTagHeadingAlign extends Command {

  private Swerve m_swerve;
  private Vision m_vision;
  private PIDController pid = new PIDController(VisionConstants.aprilTagHeadingAlignKp, VisionConstants.aprilTagHeadingAlignKi, VisionConstants.aprilTagHeadingAlignKd);

  public AprilTagHeadingAlign(Swerve m_swerve, Vision m_vision) {
    this.m_swerve = m_swerve;
    this.m_vision = m_vision;
  }

  @Override
  public void initialize() {
    m_vision.setLightID(LightID.WHITE.id);
  }

  @Override
  public void execute() {
    m_swerve.setAutoComponent(new ChassisSpeeds(0,0,pid.calculate(m_vision.getApritTagYaw(),0)));
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
