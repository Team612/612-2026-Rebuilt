// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Transfer;


public class ManualTest extends Command {
  private final Transfer m_transfer;
  private final CommandXboxController m_controller;
  private static final double SPEED = 0.2; // yes ik about the repeated instantiations

  public ManualTest(Transfer transfer, CommandXboxController controller) {
    m_transfer = transfer;
    m_controller = controller;
    addRequirements(m_transfer);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean a = m_controller.getHID().getRawButton(XboxController.Button.kA.value);
    boolean b = m_controller.getHID().getRawButton(XboxController.Button.kB.value);
    boolean x = m_controller.getHID().getRawButton(XboxController.Button.kX.value);
    boolean y = m_controller.getHID().getRawButton(XboxController.Button.kY.value);
    boolean leftBumper = m_controller.getHID().getRawButton(XboxController.Button.kLeftBumper.value);
    boolean rightBumper = m_controller.getHID().getRawButton(XboxController.Button.kRightBumper.value);

    if (rightBumper) {
      m_transfer.setMotor1(SPEED);
      m_transfer.setMotor2(SPEED);
      m_transfer.setMotor3(SPEED);
      m_transfer.setMotor4(SPEED);
    } else if (leftBumper) {
      m_transfer.setMotor1(-SPEED);
      m_transfer.setMotor2(-SPEED);
      m_transfer.setMotor3(-SPEED);
      m_transfer.setMotor4(-SPEED);
    } else {
      m_transfer.setMotor1(y ? SPEED : 0.0);
      m_transfer.setMotor2(x ? SPEED : 0.0);
      m_transfer.setMotor3(b ? SPEED : 0.0);
      m_transfer.setMotor4(a ? SPEED : 0.0);
    }
  }

  // Called once the command ends or is interrupted
  @Override
  public void end(boolean interrupted) {
    m_transfer.setMotor1(0.0);
    m_transfer.setMotor2(0.0);
    m_transfer.setMotor3(0.0);
    m_transfer.setMotor4(0.0);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
