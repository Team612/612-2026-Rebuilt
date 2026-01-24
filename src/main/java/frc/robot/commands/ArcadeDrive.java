package frc.robot.commands;


import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.TankDrive;


public class ArcadeDrive extends Command {


  private final TankDrive drivetrain;
  private final DoubleSupplier forward;
  private final DoubleSupplier turn;


  public ArcadeDrive(TankDrive drivetrain, DoubleSupplier forward, DoubleSupplier turn) {
    this.drivetrain = drivetrain;
    this.forward = forward;
    this.turn = turn;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    double forw = forward.getAsDouble();
    double tur = turn.getAsDouble();

    if (Math.abs(forw) < DriveConstants.DEADBAND)
      forw = 0;
    if (Math.abs(tur) < DriveConstants.DEADBAND)
      tur = 0;


    drivetrain.arcadeDrive(forw, tur);
  }


  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}