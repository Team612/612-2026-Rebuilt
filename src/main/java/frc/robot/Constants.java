// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    // in meters
    public static final double blueHubXPos = 4.625594;
    public static final double hubYPos = 4.034536;
    public static final double redHubXPos = 11.915394;
  }

    public static class ShooterConstants{
    public static final int shooterMotorID = 8;
    public static final int turretMotorID = 17;
    public static final int tiltMotorID = 1;
    public static final int cancoderID = 1; // this is a guess verify before running

    public static final String shooterCameraName = "PC_Camera";

    public static final double DEADBAND = 0.03;

    public static final double maxTurretSpeed = 0.1;

    public static final double kShooterHeightMeters = 0.0;

    public static final double turretCANcoderOffset = 0.0; // this is a guess

    public static final double largestTurretAngle = Math.PI/2;
    public static final double smallestTurretAngle = -Math.PI/2;

    public static final double turretKp = 0.042;
    public static final double turretKi = 0;
    public static final double turretKd = 0;
    public static final double turretGearRatio = 200.0 / 20.0;

    public static final double tiltKp = 0.1; 
    public static final double tiltKi = 0;
    public static final double tiltKd = 0;
    public static final double tiltGearRatio = 310.0 / 31.0; 

    public static final double encoderToRadians = -0.628152;
  }

  public static class VisionConstants {
    public static final int CANdleID = 1;
    public static final String shooterCamera = "PC_Camera";
    public static final Transform3d shooterCameraTransform = new Transform3d(0,0,0, new Rotation3d(0,0,0));
  }
}
