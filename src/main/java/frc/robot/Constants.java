package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    // starting pose of robot
    public static final Pose2d leftBlueTrench = new Pose2d(3.56,7.35,new Rotation2d(Math.PI));
    public static final Pose2d leftBlueBump = new Pose2d(3.56,5.55,new Rotation2d(Math.PI));
    public static final Pose2d leftBlueBumpPointAway = new Pose2d(3.56,5.55,new Rotation2d());
    public static final Pose2d blueHub = new Pose2d(3.56,4,new Rotation2d(Math.PI));
    public static final Pose2d blueHubPointAway = new Pose2d(3.56,4,new Rotation2d());
    public static final Pose2d rightBlueBump = new Pose2d(3.56,2.5,new Rotation2d(Math.PI));
    public static final Pose2d rightBlueBumpPointAway = new Pose2d(3.56,2.5,new Rotation2d());
    public static final Pose2d rightBlueTrench = new Pose2d(3.56,0.66,new Rotation2d(Math.PI));

    public static final Pose2d leftRedTrench = new Pose2d(16.5410515-3.56,0.66,new Rotation2d());
    public static final Pose2d leftRedBump = new Pose2d(16.5410515-3.56,2.5,new Rotation2d());
    public static final Pose2d leftRedBumpPointAway = new Pose2d(16.5410515-3.56,2.5,new Rotation2d(Math.PI));
    public static final Pose2d redHub = new Pose2d(16.5410515-3.56,4,new Rotation2d());
    public static final Pose2d redHubPointAway = new Pose2d(16.5410515-3.56,4,new Rotation2d(Math.PI));
    public static final Pose2d rightRedBump = new Pose2d(16.5410515-3.56,5.55,new Rotation2d());
    public static final Pose2d rightRedBumpPointAway = new Pose2d(16.5410515-3.56,5.55,new Rotation2d(Math.PI));
    public static final Pose2d rightRedTrench = new Pose2d(16.5410515-3.56,7.35,new Rotation2d());
  }
    public static class ShooterConstants{
    public static final int shooterMotorID = 7;
    public static final int turretMotorID = 6;
    public static final int tiltMotorID = 5;

    public static final String shooterCameraName = "PC_Camera";

    public static final double DEADBAND = 0.03;

    public static final double maxTurretSpeed = 0.1;

    public static final double defaultShootVolt = 6;

    public static final double shooterkV = 11.0/6000; // this is a guess
    public static final double shooterKp = 0.0; // this is a guess

    public static final double largestTurretAngle = Math.PI/2;
    public static final double smallestTurretAngle = -Math.PI/2;

    public static final double turretKp = 0.15;
    public static final double turretKi = 0.13;
    public static final double turretKd = 0.013;

    public static final double tiltKp = 3.975; 
    public static final double tiltKi = 3.01;
    public static final double tiltKd = 0;

    public static final double turretEncoderToRadians = -0.628152;

    public static final double rightLimit = -Math.PI/3;
    public static final double leftLimit = Math.PI/3;

    public static final Transform3d robotToTurret = new Transform3d(new Translation3d(-0.21, 0, 0.51), new Rotation3d());
    public static final double radiusTurretToCamera = 0.15;
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

    public static final double kWheelDiameterMeter = 0.192;
    public static final double encoderToMeters = 1; // this is a guess

    public static final double trackWidth = 0.508;
    public static final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(trackWidth);

    public static final double maxAttainableSpeed = 5.0;
    public static final double metersPerSecondToPercent = 1/maxAttainableSpeed;
    public static final double maxRadiansPerSecond = maxAttainableSpeed/(trackWidth/2);
    public static final double radiansPerSecondToPercent = 1/maxRadiansPerSecond;

    public static final double zNecessaryOffset = 1/(trackWidth/2);
    public static final double rpsScale = 512.0;
  }

  public static class IntakeConstants{
    public static final int INTAKE_MOTOR_ID = 1;

    public static final double INTAKE_SPEED = 0.85;
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

  public static class ClimbConstants {
    public static final int climbMotorID = 3; // this is a guess
    public static final double climbSpeed = 0.5; // this is a guess
    public static final double l1Distance = 1000; // this is a guess
    public static final double l2Distance = 2000; // this is a guess
    public static final double l3Distance = 3000; // this is a guess
  }

  public static class VisionConstants{
    public static final String frontCameraName = "FrontCamera";
    public static final String shooterCamera = "PC_Camera";
    public static final Transform3d shooterToCamera = new Transform3d(new Translation3d(0.15, 0.0, -0.2), new Rotation3d());

    public static final Transform3d frontCameraTransform = 
    new Transform3d(
      new edu.wpi.first.math.geometry.Translation3d(0,0,0), // this is a guess
      new edu.wpi.first.math.geometry.Rotation3d()
    );
  }
}