package frc.robot;


import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoTurretAim;
import frc.robot.commands.ManualShooterControl;
import frc.robot.commands.Shoot;
import frc.robot.commands.ZeroTurret;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.Vision;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final Vision m_vision = new Vision();
  private final TankDrive m_tankDrive = new TankDrive(new Pose2d(), m_vision);
  private final Shooter m_shooter = new Shooter();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_tankDrive.setDefaultCommand(new ArcadeDrive(m_tankDrive, m_driverController));
    m_shooter.setDefaultCommand(new ManualShooterControl(m_shooter, m_driverController));
    m_driverController.x().whileTrue(new Shoot(m_shooter));
    m_driverController.rightBumper().onTrue(new ZeroTurret(m_shooter)); 

    m_driverController.a().whileTrue(m_tankDrive.sysIdQuasistaticForward());
    m_driverController.b().whileTrue(m_tankDrive.sysIdQuasistaticReverse());
    m_driverController.x().whileTrue(m_tankDrive.sysIdDynamicForward());
    m_driverController.y().whileTrue(m_tankDrive.sysIdDynamicReverse());

  }

  public Command getAutonomousCommand() {
    //Drivetrain Commands

    // try{
    //     // Load the path you want to follow using its name in the GUI
    //     PathPlannerPath path = PathPlannerPath.fromPathFile("TestPath");

    //     // Create a path following command using AutoBuilder. This will also trigger event markers.
    //     return AutoBuilder.followPath(path);
    // } catch (Exception e) {
    //     DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    //     return Commands.none();
    // }

    // Shooter Auto Commands
    return new AutoTurretAim(m_shooter);
  }
}
