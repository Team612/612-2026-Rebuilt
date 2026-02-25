package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kGunnerControllerPort = 1;

    // in meters
    public static final double blueHubXPos = 4.625594;
    public static final double hubYPos = 4.034536;
    public static final double redHubXPos = 11.915394;
  }

    public static class ShooterConstants{
    public static final int shooterMotorID = 7;
    public static final int turretMotorID = 6;
    public static final int tiltMotorID = 5;

    public static final String shooterCameraName = "PC_Camera";

    public static final double DEADBAND = 0.03;

    public static final double maxTurretSpeed = 0.1;

    public static final double defaultShootSpeed = 0.5;

    public static final double largestTurretAngle = Math.PI/2;
    public static final double smallestTurretAngle = -Math.PI/2;

    public static final double turretKp = 0.15;
    public static final double turretKi = 0.13;
    public static final double turretKd = 0.013;

    public static final double tiltKp = 3.975; 
    public static final double tiltKi = 3.01;
    public static final double tiltKd = 0;

    public static final double turretEncoderToRadians = -0.628152;

    public static final int rightLimitDIO = 0;
    public static final int leftLimitDIO = 0;
    public static final double rightLimit = -Math.PI/3;
    public static final double leftLimit = Math.PI/3;
  }
  
  public static class DriveConstants {
    public static final int leftMotorID = 0;
    public static final int rightMotorID = 5;
    public static final int leftMotor2ID = 8;
    public static final int rightMotor2ID = 1;

    public static final int gyroID = 0;

    public static final double xPercent = 1;
    public static final double zPercent = 1;

    public static final double DEADBAND = 0.05;

    public static final double trackWidth = 0.508;
    public static final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(trackWidth);

    public static final double maxAttainableSpeed = 5.0;
    public static final double metersPerSecondToPercent = 1/maxAttainableSpeed;
    public static final double maxRadiansPerSecond = maxAttainableSpeed/(trackWidth/2);
    public static final double radiansPerSecondToPercent = 1/maxRadiansPerSecond;

    public static final double zNecessaryOffset = 1/(trackWidth/2);
  }

  public static class IntakeConstants{
    public static final int INTAKE_MOTOR_ID = 1; 

    public static final double INTAKE_SPEED = 0.85;  // this is a guess
    public static final double OUTTAKE_SPEED = -0.65;  // this is a guess
  }

  public static class TransferConstants{
    public static final int hopperTopID = 3;
    public static final int hopperBottomID = 2;
    public static final int feedID = 4;

    public static final double hopperTopSpeed = 0.5;
    public static final double hopperBottomSpeed = -0.5;
    public static final double feedSpeed = 0.5;

    public static final int pauseTime = 50;
  }

  public static class VisionConstants{
    public static final String frontCameraName = "FrontCamera";
    public static final String shooterCamera = "PC_Camera";
    public static final Transform3d shooterToCamera = new Transform3d(new Translation3d(0.15, 0.0, -0.2), new Rotation3d());
 
    public static final Transform3d frontCameraTransform = 
    new Transform3d(
      new edu.wpi.first.math.geometry.Translation3d(
        edu.wpi.first.math.util.Units.inchesToMeters(0), // this is a guess
        edu.wpi.first.math.util.Units.inchesToMeters(0), // this is a guess
        edu.wpi.first.math.util.Units.inchesToMeters(0)),// this is a guess
      new edu.wpi.first.math.geometry.Rotation3d()
    );
  }
  
}