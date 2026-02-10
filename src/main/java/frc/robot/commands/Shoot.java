// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {

  private Shooter m_shooter;

  private boolean red;
  private double hubXpos;

    // this is a stand in for the drivetrain replace when done
  public Pose2d getPose(){
    return new Pose2d(0,4.034536,new Rotation2d());
  } 

  public Shoot(Shooter m_shooter) {
    this.m_shooter = m_shooter;
  }

  @Override
  public void initialize() {
    if (DriverStation.getAlliance().isPresent()) {
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
    if ((m_shooter.calculateShootingAnglesWithOfficialOffset()[0] == -1) && (m_shooter.calculateShootingAnglesWithOfficialOffset()[1] == -1)){
      Pose2d robotPos = getPose();
  
      double xdiff = hubXpos - robotPos.getX();
      double ydiff = OperatorConstants.hubYPos - robotPos.getY();
      
      m_shooter.setShooterMotor(m_shooter.getRegressionModelDutyCycle(Math.sqrt(xdiff*xdiff+ydiff*ydiff)));
    }
    else{
      m_shooter.setShooterMotor(m_shooter.getRegressionModelDutyCycle(m_shooter.calculateShootingAnglesWithOfficialOffset()[1]));
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.setShooterMotor(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
