package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoTurretAim;
import frc.robot.commands.AutonomousRoutine;
import frc.robot.commands.Feed;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.Shoot;
import frc.robot.commands.ZeroTurret;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_gunnerController = new CommandXboxController(OperatorConstants.kGunnerControllerPort);

  private final Vision m_vision = new Vision();
  private final TankDrive m_tankDrive = new TankDrive(new Pose2d(), m_vision);
  private final Shooter m_shooter = new Shooter();
  private final Transfer m_transfer = new Transfer();
  private final Intake m_intake = new Intake();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_tankDrive.setDefaultCommand(new ArcadeDrive(m_tankDrive, m_driverController));

    m_shooter.setDefaultCommand(new AutoTurretAim(m_shooter));
    m_gunnerController.x().whileTrue(new ParallelCommandGroup(
      new Shoot(m_shooter),
      new Feed(m_transfer)
    ));
    m_gunnerController.y().whileTrue(new IntakeBall(m_intake)); 
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      new ZeroTurret(m_shooter),
      new AutonomousRoutine(m_tankDrive)
    );
  }
}
