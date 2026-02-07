package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;

public class ManualShooterControl extends Command {

  private Shooter m_shooter;
  private CommandXboxController controller;

  public ManualShooterControl(Shooter m_shooter, CommandXboxController controller) {
    this.m_shooter = m_shooter;
    this.controller = controller;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    // double x = controller.getRawAxis(1);
    // double y = controller.getRawAxis(0);
    // double rad = -Math.atan2(x,y);

    // rad -= Math.PI/2;
    // if (rad < -Math.PI)
    //   rad+= 2 * Math.PI;
    
    // m_shooter.setTurretPos(rad);
    // if (m_shooter.shooterHasTag()){
    //   // System.out.println(- (m_shooter.tagOff()-(Math.PI/2)));
    //   m_shooter.setTurretPos(m_shooter.getCurrentTurretAngle() - (m_shooter.tagOff()-(Math.PI/2)));
    // }

    if (controller.getHID().getAButton()){
      m_shooter.setShooterMotor(-0.5895);
    }
    else
      m_shooter.setShooterMotor(0.0);

    m_shooter.setTiltMotor(controller.getLeftY());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}