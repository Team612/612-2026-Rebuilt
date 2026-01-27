// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ShooterVision;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final ShooterVision m_shooterVision = new ShooterVision();
  private final Shooter m_shooter = new Shooter();

  public RobotContainer() {
    configureBindings();
    DataLogManager.start();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return null;
  }
}
