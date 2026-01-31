// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousRoutine;
import frc.robot.subsystems.TankDrive;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private Pigeon2 m_gyro = new Pigeon2(Constants.DriveConstants.gyroID);
  private TankDrive m_tankDrive = new TankDrive(m_gyro);
  private final AutonomousRoutine routine = new AutonomousRoutine(m_tankDrive);
  // private boolean red;
  // if (DriverStation.getAlliance().isPresent()) {
  //   if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
  //     red = true;
  // }

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {    
    new ArcadeDrive(m_tankDrive, 
    () -> m_driverController.getLeftY(), 
    () -> m_driverController.getLeftX());
  }

  public Command getAutonomousCommand() {
    return routine;
  }
}