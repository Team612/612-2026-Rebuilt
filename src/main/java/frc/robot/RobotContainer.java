// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArcadeClimb;
import frc.robot.commands.L1;
import frc.robot.commands.L1Descend;
import frc.robot.commands.L2;
import frc.robot.commands.L2Descend;

import frc.robot.subsystems.Climb;

public class RobotContainer {

  private final CommandXboxController climbController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final Climb climb = new Climb();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    climb.setDefaultCommand(new ArcadeClimb(climb, climbController));
  }

  public Command getAutonoumousCommand() {
    return new SequentialCommandGroup(
      //new ArcadeClimb(climb, climbController)
      //new L1(climb).withTimeout(10),
      //new L1Descend(climb).withTimeout(10)
      //new L2(climb).withTimeout(10);
      //new L2Descend(climb).withTimeout(10);
    );
  }
}
