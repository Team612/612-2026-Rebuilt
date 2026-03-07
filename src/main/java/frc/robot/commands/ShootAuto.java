package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TankDrive;

public class ShootAuto extends Command {

  private Shooter m_shooter;
  private TankDrive m_tankDrive;

  private boolean red;
  private double hubXpos;

  public ShootAuto(Shooter m_shooter, TankDrive m_tankDrive){
    this.m_shooter = m_shooter;
    this.m_tankDrive = m_tankDrive;
  }

  @Override
  public void initialize() {
    if (DriverStation.getAlliance().isPresent()){
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
        red = true;
    }
    if (red)
      hubXpos = OperatorConstants.redHubXPos;
    else 
      hubXpos = OperatorConstants.blueHubXPos;
  }

  @Override
  public void execute() {
    m_shooter.setShooterRPM(3000);
  }

  @Override
  public void end(boolean interrupted){
    m_shooter.setShooterRPM(0);
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}