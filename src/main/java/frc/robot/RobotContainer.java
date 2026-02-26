package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.TankDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final TankDrive m_tankDrive = new TankDrive(OperatorConstants.BlueHubPointAway);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_tankDrive.setDefaultCommand(new ArcadeDrive(m_tankDrive, m_driverController));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
