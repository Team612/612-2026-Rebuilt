package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.TransferConstants;

public class UpdateData extends Command {

  private final Telemetry m_tele; 


  public UpdateData(Telemetry telemetry) {
    m_tele = telemetry;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_tele.updateData();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
