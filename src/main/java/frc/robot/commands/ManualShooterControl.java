package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class ManualShooterControl extends Command {

  private Shooter m_shooter;
  private JoystickButton machineGunButton;
  private Joystick variable;

  public ManualShooterControl(Shooter m_shooter, Joystick variable, JoystickButton machineGunButton) {
    this.m_shooter = m_shooter;
    this.variable = variable;
    this.machineGunButton = machineGunButton;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (machineGunButton.getAsBoolean())
      m_shooter.setShooterRPM(ShooterConstants.defaultShootRPM);
    else
      m_shooter.setShooterRPM(0);

    m_shooter.setTurretMotor(-variable.getRawAxis(0)*0.05);
    m_shooter.setTiltMotor(variable.getRawAxis(1));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}