package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TestTransfer;
import frc.robot.subsystems.Transfer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_gunnerController = new CommandXboxController(OperatorConstants.kGunnerControllerPort);

  private final Transfer m_transfer = new Transfer();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_transfer.setDefaultCommand(new TestTransfer(m_transfer, m_gunnerController));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
