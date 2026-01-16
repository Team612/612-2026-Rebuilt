package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve;

public class ArcadeDrive extends Command {

  private Swerve m_swerve;
  private CommandXboxController controller;
  private boolean red = false;

  public ArcadeDrive(Swerve m_swerve, CommandXboxController  controller) {
    this.m_swerve = m_swerve;
    this.controller = controller;
    addRequirements(m_swerve);
  }

  @Override
  public void initialize() {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
        red = true;
    }
  }

  @Override
  public void execute() {
    double x = -controller.getRawAxis(1);
    double y = -controller.getRawAxis(0);
    double zRot = -controller.getRawAxis(4);

    if (Math.abs(x) < DriveConstants.DEADBAND) x = 0;
    if (Math.abs(y) < DriveConstants.DEADBAND) y = 0;
    if (Math.abs(zRot) < DriveConstants.DEADBAND) zRot = 0;

    x *= DriveConstants.xPercent;
    if (red)
      x *= -1;
    y *= DriveConstants.yPercent;
    zRot *=DriveConstants. zNecessaryOffset;

    if (controller.rightBumper().getAsBoolean())
      m_swerve.drive(new ChassisSpeeds(x,y,zRot));
    else
      m_swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(x,y,zRot, m_swerve.getHeading()));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
