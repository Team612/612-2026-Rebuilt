// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousRoutine;
import frc.robot.subsystems.TankDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;


public class RobotContainer {
  private TankDrive m_tankDrive = new TankDrive();

  // private boolean red;
  // if (DriverStation.getAlliance().isPresent()) {
  //   if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
  //     red = true;
  // }

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {

  try {
    AutoBuilder.configure(
        m_tankDrive::getPose,
        m_tankDrive::resetPose,
        m_tankDrive::getRobotRelativeSpeeds,
        m_tankDrive::driveRobotRelative,
        new PPLTVController(0.02),
        RobotConfig.fromGUISettings(),
        () -> false,
        m_tankDrive
    );
  } catch (Exception e) {
    System.err.println("PathPlanner AutoBuilder failed to configure");
    e.printStackTrace();
  }

  configureBindings();
}



  private void configureBindings() {    
    new ArcadeDrive(m_tankDrive, 
    () -> m_driverController.getLeftY(), 
    () -> m_driverController.getLeftX());
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("StraightTest"); // name from GUI
  }
}