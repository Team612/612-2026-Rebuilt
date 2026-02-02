package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.TankDrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final TankDrive m_tankDrive = new TankDrive(new Pose2d());

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_tankDrive.setDefaultCommand(new ArcadeDrive(m_tankDrive, m_driverController));
  }

  public Command getAutonomousCommand() {
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("TestPath");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }
}
