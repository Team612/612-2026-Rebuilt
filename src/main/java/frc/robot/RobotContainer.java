package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoTurretAim;
import frc.robot.commands.ManualShooterControl;
import frc.robot.commands.ResetEncoders;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final Shooter m_shooter = new Shooter();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_shooter.setDefaultCommand(new ManualShooterControl(m_shooter, m_driverController));
    m_driverController.a().onTrue(new ResetEncoders(m_shooter));
  }

  public Command getAutonomousCommand() {
    return new AutoTurretAim(m_shooter);
  }
}
