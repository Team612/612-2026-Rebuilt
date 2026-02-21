package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoTurretAim;
import frc.robot.commands.FeedAndShoot;
import frc.robot.commands.ManualIntakeHopper;
import frc.robot.commands.ManualShooterControl;
import frc.robot.commands.ZeroTurret;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.Transfer;

import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_gunnerController = new CommandXboxController(OperatorConstants.kGunnerControllerPort);

  private final TankDrive m_tankDrive = new TankDrive();
  private final Shooter m_shooter = new Shooter();
  private final Transfer m_transfer = new Transfer();
  private final Intake m_intake = new Intake();
  
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_tankDrive.setDefaultCommand(new ArcadeDrive(m_tankDrive, m_driverController));
    
    m_shooter.setDefaultCommand(new ManualShooterControl(m_shooter, m_gunnerController));
    m_gunnerController.rightBumper().onTrue(new ZeroTurret(m_shooter));
    m_gunnerController.x().whileTrue(new ManualIntakeHopper(m_intake, m_transfer));
    m_gunnerController.b().whileTrue(new FeedAndShoot(m_transfer, m_shooter));
  }

  public Command getAutonomousCommand() {
    return new AutoTurretAim(m_shooter);
  }
}
