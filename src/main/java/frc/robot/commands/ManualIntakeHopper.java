package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;

public class ManualIntakeHopper extends Command {

  private final Intake m_intake;
  private final Transfer m_transfer;
  
  public ManualIntakeHopper(Intake m_intake, Transfer transfer) {
    this.m_intake = m_intake;
    this.m_transfer = transfer;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_intake.setMotor(IntakeConstants.INTAKE_SPEED);
    // m_transfer.setHopperTop(TransferConstants.TRANSFER_SPEED);
    // m_transfer.setHopperBottom(-TransferConstants.TRANSFER_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.setMotor(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
