package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TransferConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;

public class Agitate extends Command {

  private Transfer m_transfer;
  private Intake m_intake;
  private int timer;

  public Agitate(Transfer m_transfer, Intake m_intake) {
    this.m_transfer = m_transfer;
    this.m_intake = m_intake;
    addRequirements(m_transfer, m_intake);
  }

  @Override
  public void initialize() {
    timer = 0;
    m_intake.setJammerSpeed(SmartDashboard.getNumber("AgitatorIntake", -0.2));
  }

  @Override
  public void execute() {
    timer++;
    if ((timer % (TransferConstants.agitateTime*2)) < TransferConstants.agitateTime){
      m_transfer.setHopperTop(-SmartDashboard.getNumber("Hopper 1", 0.2));
      m_transfer.setHopperBottom(-SmartDashboard.getNumber("Hopper 2", -0.3));
    }

    else{
      m_transfer.setHopperTop(SmartDashboard.getNumber("Hopper 1", 0.2));
      m_transfer.setHopperBottom(SmartDashboard.getNumber("Hopper 2", -0.3));
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_transfer.setHopperTop(0);
    m_transfer.setHopperBottom(0);
    m_intake.setJammerSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
