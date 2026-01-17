// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.SetEncoders;
import frc.robot.commands.VisionMenu;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private Swerve m_swerve = new Swerve();
  private Vision m_vision = new Vision();

  public RobotContainer() {
    m_swerve.setDefaultCommand(new ArcadeDrive(m_swerve, m_driverController));
    m_vision.setDefaultCommand(new VisionMenu(m_vision, m_driverController));
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.leftBumper().onTrue(new SetEncoders(m_swerve, new Pose2d()));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
