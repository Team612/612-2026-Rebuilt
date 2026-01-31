package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TestHopper;
import frc.robot.commands.TestIndexer;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  // new data fields for some objects 
  private Hopper m_hopper = new Hopper();
  private Indexer m_indexer = new Indexer();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // manual controls for hopper/indexer PLACEHOLDERS pls
    // A: hopper forward B: hopper reverse to make things work
    m_driverController.a().whileTrue(new RunCommand(() -> m_hopper.runForward(), m_hopper));
    m_driverController.b().whileTrue(new RunCommand(() -> m_hopper.runReverse(), m_hopper));

    // X: indexer forward, Y: indexer reverse
    m_driverController.x().whileTrue(new RunCommand(() -> m_indexer.runForward(), m_indexer));
    m_driverController.y().whileTrue(new RunCommand(() -> m_indexer.runReverse(), m_indexer));

    // Start: TestHopper, Back: TestIndexer
    m_driverController.start().onTrue(new TestHopper(m_hopper));
    m_driverController.back().onTrue(new TestIndexer(m_indexer));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
