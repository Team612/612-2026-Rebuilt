package frc.robot;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoTurretAim;

import frc.robot.commands.AutonomousRoutine;
import frc.robot.commands.Feed;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.ManualShooterControl;
import frc.robot.commands.ReverseAllMotors;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootAuto;
import frc.robot.commands.ZeroTurret;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Vision;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // FLIP WITH CONTROLLERS
  private final CommandXboxController m_gunnerController = new CommandXboxController(OperatorConstants.kGunnerControllerPort);
  // private static Joystick m_buttonGunner = new Joystick(OperatorConstants.kGunnerPortButtons);
  // private static Joystick m_variableGunner = new Joystick(OperatorConstants.kGunnerPortVariable);
  // private static JoystickButton machineGunButton = new JoystickButton(m_buttonGunner, OperatorConstants.machineGunButtonID);
  // private static JoystickButton intakeButton = new JoystickButton(m_buttonGunner, OperatorConstants.intakeButtonID);
  // private static JoystickButton resetButton = new JoystickButton(m_buttonGunner, OperatorConstants.resetButtonID);
  // private static JoystickButton zeroCountButton = new JoystickButton(m_buttonGunner, OperatorConstants.zeroCountButtonID);
  // private static JoystickButton dumpButton = new JoystickButton(m_buttonGunner, OperatorConstants.dumpButtonID);

  private final Vision m_vision = new Vision();
  private final Shooter m_shooter = new Shooter();
  private final TankDrive m_tankDrive = new TankDrive(OperatorConstants.blueHub, m_vision, m_shooter);
  private final Transfer m_transfer = new Transfer();
  private final Intake m_intake = new Intake();

  private static boolean manualMode = false;

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_tankDrive.setDefaultCommand(new ArcadeDrive(m_tankDrive, m_driverController));

    // FLIP WITH CONTROLLERS
    if (manualMode)
      m_shooter.setDefaultCommand(new ManualShooterControl(m_shooter, () -> -m_gunnerController.getLeftX(), () -> m_gunnerController.getRightY(), () -> m_gunnerController.getHID().getBButton()));
    else
      m_shooter.setDefaultCommand(new AutoTurretAim(m_shooter, m_tankDrive));

    m_gunnerController.leftBumper().onTrue(new InstantCommand(() -> {
      manualMode = !manualMode;
      m_shooter.getCurrentCommand().cancel();

      if (manualMode)
        m_shooter.setDefaultCommand(new ManualShooterControl(m_shooter, () -> -m_gunnerController.getLeftX(), () -> m_gunnerController.getRightY(), () -> m_gunnerController.getHID().getBButton()));
      else
        m_shooter.setDefaultCommand(new AutoTurretAim(m_shooter, m_tankDrive));
    }));
    m_gunnerController.y().whileTrue(new ReverseAllMotors(m_transfer, m_shooter));
    m_gunnerController.b().and(() -> !manualMode).whileTrue(new Shoot(m_shooter, m_tankDrive));
    m_gunnerController.b().or(m_gunnerController.x()).whileTrue(new Feed(m_transfer, () -> m_gunnerController.getHID().getXButton()));
    m_gunnerController.a().whileTrue(new IntakeBall(m_intake));
    // if (manualMode)
    //   m_shooter.setDefaultCommand(new ManualShooterControl(m_shooter, () -> -m_variableGunner.getRawAxis(0), () -> m_variableGunner.getRawAxis(1), () -> machineGunButton.getAsBoolean()));
    // else
    //   m_shooter.setDefaultCommand(new AutoTurretAim(m_shooter, m_tankDrive));

    // zeroCountButton.onTrue(new InstantCommand(() -> {
    //   manualMode = !manualMode;
    //   m_shooter.getCurrentCommand().cancel();

    //   if (manualMode)
    //     m_shooter.setDefaultCommand(new ManualShooterControl(m_shooter, () -> -m_variableGunner.getRawAxis(0), () -> m_variableGunner.getRawAxis(1), () -> machineGunButton.getAsBoolean()));
    //   else
    //     m_shooter.setDefaultCommand(new AutoTurretAim(m_shooter, m_tankDrive));
    // }));
    // dumpButton.whileTrue(new ReverseAllMotors(m_transfer, m_shooter));
    // machineGunButton.and(() -> !manualMode).whileTrue(new Shoot(m_shooter, m_tankDrive));
    // machineGunButton.or(resetButton).whileTrue(new Feed(m_transfer, () -> resetButton.getAsBoolean()));
    // intakeButton.whileTrue(new IntakeBall(m_intake));
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      new ZeroTurret(m_shooter),
      new AutonomousRoutine(m_tankDrive),
      new ParallelCommandGroup(new AutoTurretAim(m_shooter, m_tankDrive), new ShootAuto(m_shooter, m_tankDrive), new IntakeBall(m_intake), new Feed(m_transfer, () -> false))
    );
  }
}