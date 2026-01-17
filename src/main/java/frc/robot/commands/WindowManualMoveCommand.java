// src/main/java/frc/robot/commands/HomeWindowMotorCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class WindowManualMoveCommand extends Command {
  /** Creates a new WindowManualMoveCommand. */

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  private final Shooter windowMotor;
  private final double speed;

  public WindowManualMoveCommand(Shooter window, double s) {
    windowMotor = window;
    speed = s;
    addRequirements(windowMotor
    );
  }
  @Override
  public void initialize() {
      // Optional: clear encoder before homing
      // windowMotor.resetEncoder();
  }

  @Override
  public void execute() {
      // Drive toward the limit switch
      windowMotor.setShooterSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
      // Stop and make sure encoder is zeroed
      windowMotor.stopShooterMotor();
  }

  @Override
  public boolean isFinished() {
      // Finish when limit switch is hit
      return false;
    }
}
