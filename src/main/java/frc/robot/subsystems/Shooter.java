// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX shooterMotor;
  private SparkMax tiltMotor;
  private SparkMax turretMotor;

  private SparkClosedLoopController turretController;
  private SparkClosedLoopController tiltController;
  public Shooter() {
    tiltMotor = new SparkMax(Constants.ShooterConstants.tiltId, MotorType.kBrushless);
    turretMotor = new SparkMax(Constants.ShooterConstants.turretId, MotorType.kBrushless);
    shooterMotor = new TalonFX(Constants.ShooterConstants.shooterId);

    turretController = turretMotor.getClosedLoopController();
    tiltController = tiltMotor.getClosedLoopController();
  }

  public void setShooterSpeed(int speed){
    shooterMotor.set(speed);
  }
  
   

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
