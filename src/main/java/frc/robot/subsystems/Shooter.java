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
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private SparkMax shooterMotor;
  private SparkMax tiltMotor;
  private SparkMax turretMotor;
  private final RelativeEncoder tiltEncoder;
  private final DigitalInput tiltLimitSwitch;

  private static final double defaultPos = 0.0;

  private PIDController tiltPID;
  private double targetPosition = 0.0;
  private boolean positionModeEnabled = false;
  // Turret (yaw) control
  private PIDController turretPID;
  private double turretTargetAngleDeg = 0.0; // absolute degrees: 0 = straight ahead
  private boolean turretPositionModeEnabled = false;
  private boolean turretSweeping = false;
  private int sweepDirection = 1; // 1 == clockwise, -1 == counter-clockwise
  private static final double turretSweepOpenLoopPower = 0.05; // as requested
  private static final double turretGearRatio = 10.0; // motor:output = 10:1 (motor rotates 10x output)
  private final com.revrobotics.RelativeEncoder turretEncoder;
  private final DoubleLogEntry velocityLogEntry;
  public Shooter() {
    DataLog log = DataLogManager.getLog();

    // Create the log entry ONCE
    velocityLogEntry = new DoubleLogEntry(log, "/Shooter/Velocity");
    tiltMotor = new SparkMax(ShooterConstants.tiltId, MotorType.kBrushed);
    turretMotor = new SparkMax(ShooterConstants.turretId, MotorType.kBrushless);
    shooterMotor = new SparkMax(ShooterConstants.shooterId, MotorType.kBrushless);

    tiltEncoder = tiltMotor.getEncoder();
    tiltEncoder.setPosition(0.0);
    tiltLimitSwitch = new DigitalInput(ShooterConstants.tiltLimitSwitchId);

    tiltPID = new PIDController(ShooterConstants.tildKp, ShooterConstants.tildKi, ShooterConstants.tildKd);
    tiltPID.setTolerance(0.5);
    turretPID = new PIDController(ShooterConstants.turretKp, ShooterConstants.turretKi, ShooterConstants.turretKd);
    turretPID.setTolerance(1.0); // 1 degree tolerance
    turretEncoder = turretMotor.getEncoder();
    // assume turret starts facing straight ahead -> set encoder zero
    turretEncoder.setPosition(0.0);
  }

  public void setShooterSpeed(double speed){
    shooterMotor.set(speed);
  }
  
  public void stopShooterMotor() {
    positionModeEnabled = false;
    shooterMotor.stopMotor();
}


public void setTiltPower(double power) {
    positionModeEnabled = false;
    double safePower = applyLimitSwitchSafety(power);
    tiltMotor.set(safePower);
}

public void stopTilt() {
    positionModeEnabled = false;
    tiltMotor.stopMotor();
}

  public double getShooterVelocity(){
    return shooterMotor.getEncoder().getVelocity();
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
public void tiltGoToPosition(double target) {
    targetPosition = target;
    tiltPID.setSetpoint(target+defaultPos);
    positionModeEnabled = true;
}

/** Returns true when the motor reaches the target. */
public boolean atTarget() {
    return tiltPID.atSetpoint();
}

  // -------------------- Turret helpers --------------------
  /** Convert degrees (output shaft) to motor rotations using gearing. */
  private double degreesToMotorRotations(double degrees) {
    double outputRotations = degrees / 360.0;
    return outputRotations * turretGearRatio;
  }

  /** Convert motor rotations to output shaft degrees. */
  private double motorRotationsToDegrees(double motorRotations) {
    double outputRotations = motorRotations / turretGearRatio;
    return outputRotations * 360.0;
  }

  /** Move turret to an absolute angle in degrees (0 = straight ahead). */
  public void turretGoToAbsoluteAngle(double angleDeg) {
    turretTargetAngleDeg = angleDeg;
    // normalize to [-180,180]
    if (turretTargetAngleDeg > 180.0) turretTargetAngleDeg = ((turretTargetAngleDeg + 180) % 360) - 180;
    if (turretTargetAngleDeg <= -180.0) turretTargetAngleDeg = ((turretTargetAngleDeg - 180) % 360) + 180;
    double setpointMotorRot = degreesToMotorRotations(turretTargetAngleDeg);
    turretPID.setSetpoint(setpointMotorRot);
    turretPositionModeEnabled = true;
    turretSweeping = false; // stop sweeping when we explicitly aim
  }

  public boolean atTurretTarget() {
    if (!turretPositionModeEnabled) return false;
    return turretPID.atSetpoint();
  }

  public void startTurretSweep() {
    turretSweeping = true;
    turretPositionModeEnabled = false;
    sweepDirection = 1;
  }

  public void stopTurretSweep() {
    turretSweeping = false;
    turretMotor.stopMotor();
  }

  public void manualTurretPower(double power) {
    turretPositionModeEnabled = false;
    turretSweeping = false;
    turretMotor.set(power);
  }

  public void stopTurretMotor() {
    turretPositionModeEnabled = false;
    turretSweeping = false;
    turretMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        if (isLimitSwitchPressed()) {
          stopShooterMotor();
          resetTiltEncoder();
      }
    velocityLogEntry.append(getShooterVelocity());
// Position control (only runs if enabled and not stopped above)
      if (positionModeEnabled) {

  }

    // Run turret closed-loop control if enabled
    if (turretPositionModeEnabled) {
      double currentMotorPos = turretEncoder.getPosition();
      double output = turretPID.calculate(currentMotorPos);
      // PID output is motor rotations/sec approx — scale/clamp to [-1,1]
      // simple clamp: assume max reasonable command magnitude
      if (output > 1.0) output = 1.0;
      if (output < -1.0) output = -1.0;
      turretMotor.set(output);
    }

    // If sweeping, run a slow open-loop sweep but stop/reverse at +/-180 degrees
    if (turretSweeping) {
      // get current output shaft angle
      double currentAngle = motorRotationsToDegrees(turretEncoder.getPosition());
      // normalize currentAngle into [-180,180]
      if (currentAngle > 180.0) currentAngle = ((currentAngle + 180) % 360) - 180;
      if (currentAngle <= -180.0) currentAngle = ((currentAngle - 180) % 360) + 180;

      // reverse direction at the limits to avoid continuous rotation and cable tangles
      if (currentAngle >= 180.0 - 1.0) {
        sweepDirection = -1;
      } else if (currentAngle <= -180.0 + 1.0) {
        sweepDirection = 1;
      }
      turretMotor.set(turretSweepOpenLoopPower * sweepDirection);
    }

    // Vision-based detection and aiming
    frc.robot.subsystems.ShooterVision vision = frc.robot.subsystems.ShooterVision.getVisionInstance();
    if (vision != null && vision.shooterHasTag()) {
      // stop any sweeping and aim
      stopTurretSweep();
      double[] angles = vision.calculateShootingAnglesWithOfficialOffset();
      double yawDeg = angles[0];
      double pitchDeg = angles[1];

      // move turret (yaw) closed-loop to yawDeg
      turretGoToAbsoluteAngle(yawDeg);

      // move tilt (pitch) — reuse tiltGoToPosition. Assume tilt expects degrees.
      tiltGoToPosition(pitchDeg);
    }
}
}
