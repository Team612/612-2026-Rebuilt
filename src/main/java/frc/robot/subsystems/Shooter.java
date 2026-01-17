// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private SparkMax shooterMotor;
  private SparkMax tiltMotor;
  private SparkMax turretMotor;
  private final RelativeEncoder tiltEncoder;
  private final DigitalInput tiltLimitSwitch;

  private static final double kP = 0.04;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double defaultPos = 0.0;

  private PIDController tiltPID;
  private double targetPosition = 0.0;
  private boolean positionModeEnabled = false;

  public Shooter() {
    tiltMotor = new SparkMax(Constants.ShooterConstants.tiltId, MotorType.kBrushed);
    turretMotor = new SparkMax(Constants.ShooterConstants.turretId, MotorType.kBrushless);
    shooterMotor = new SparkMax(Constants.ShooterConstants.shooterId, MotorType.kBrushless);

    tiltEncoder = tiltMotor.getEncoder();
    tiltEncoder.setPosition(0.0);
    tiltLimitSwitch = new DigitalInput(Constants.ShooterConstants.tiltLimitSwitchId);

    tiltPID = new PIDController(kP, kI, kD);
    tiltPID.setTolerance(0.5);
  }

  public void setShooterSpeed(double speed){
    shooterMotor.set(speed);
  }
  
  public void stopShooterMotor() {
    positionModeEnabled = false;
    shooterMotor.stopMotor();
}

/** Manually reset the encoder to 0. */
public void resetTiltEncoder() {
    tiltEncoder.setPosition(defaultPos);
}

public double getTiltPosition() {
    return tiltEncoder.getPosition()-defaultPos;
}

public double getTiltVelocity() {
    return tiltEncoder.getVelocity();
}

//limit switch thing

/** Returns true if the limit switch is pressed. */
public boolean isLimitSwitchPressed() {
    // NOTE: many switches are wired "normally closed" so you might need '!' here.
    return tiltLimitSwitch.get();  // assume pressed = circuit closed = returns false → invert
}

private double applyLimitSwitchSafety(double requestedPower) {
    if(isLimitSwitchPressed() && requestedPower > 0.0) {
        return 0.0;
    }
    if (getTiltPosition() > 1 && requestedPower < 0.0) {
        return 0.0;
    }
    return requestedPower;
}

//move to position 
/** Move motor to a specific encoder position. */
public void goToPosition(double target) {
    targetPosition = target;
    tiltPID.setSetpoint(target+defaultPos);
    positionModeEnabled = true;
}

/** Returns true when the motor reaches the target. */
public boolean atTarget() {
    return tiltPID.atSetpoint();
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        if (isLimitSwitchPressed()) {
          stopShooterMotor();
          resetTiltEncoder();
      }

      System.out.println(isLimitSwitchPressed());
      // Position control (only runs if enabled and not stopped above)
      if (positionModeEnabled) {

  }
}
}
