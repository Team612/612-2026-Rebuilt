// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import com.fasterxml.jackson.databind.module.SimpleSerializers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Shooter;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootUntilEmpty extends Command {

  private Shooter m_shooter;

  private boolean red;
  Timer timerA = new Timer();
  
  private double hubXpos;

    // this is a stand in for the drivetrain replace when done
  public Pose2d getPose(){
    return new Pose2d(0,4.034536,new Rotation2d());
  } 

  public ShootUntilEmpty(Shooter m_shooter) {
    this.m_shooter = m_shooter;
    timerA.start();
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

      double distance = Math.sqrt(xdiff*xdiff+ydiff*ydiff);
      m_shooter.setShooterVelocity(m_shooter.getRegressionModelRPM(distance));


      if (m_shooter.getRegressionModelRPM(distance)-80 > m_shooter.getRPM()) {
        timerA.reset();
      }
    }
    else{
      m_shooter.setShooterVelocity(m_shooter.getRegressionModelRPM(m_shooter.calculateShootingAnglesWithOfficialOffset()[1]));
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.setShooterVelocity(0);
  }

  @Override
  public boolean isFinished() {
    return timerA.get() > 5.0;
  }
}