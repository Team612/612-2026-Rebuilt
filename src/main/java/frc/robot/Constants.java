package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    public static final double abosulteMaxCurrentLimit = 100; // keep in mind in practice the highest it will realistically go it 80% of this
    public static final double steerSupplyCurrent = 5;
    public static final double driveSupplyCurrent = (abosulteMaxCurrentLimit-steerSupplyCurrent*4)/4;

    public static final double trackWidth = 0.555;
    public static final double wheelBase = 0.550;

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
    public static final double radiusInMeters = Math.sqrt(trackWidth*trackWidth+wheelBase*wheelBase)/2;

    public static final double maxMetersPerSecondSpeed = 5;
    public static final double metersPerSecondToPercent = 1/maxMetersPerSecondSpeed;
    public static final double maxRadiansPerSecondSpeed = maxMetersPerSecondSpeed/radiusInMeters;
    public static final double radiansPerSecondToPercent = 1/maxRadiansPerSecondSpeed;

    public static final double rotationsToMeters = 0.0653451271947;

    public static final double xPercent = 1;
    public static final double yPercent = 1;
    public static final double zPercent = 1;

    public static final double zNecessaryOffset = zPercent/radiusInMeters;

    public static final int gyroID = 0;

    public static final double DEADBAND = 0.05;

    public static final double kp = 0.5;

    public static final double frontLEncoderOffset = -0.01;
    public static final int frontLSteerMotorID = 7;
    public static final int frontLDriveMotorID = 6;
    public static final int frontLCANcoderID = 4;

    public static final double frontREncoderOffset = -0.473;
    public static final int frontRSteerMotorID = 5;
    public static final int frontRDriveMotorID = 4;
    public static final int frontRCANcoderID = 3;

    public static final double backLEncoderOffset = 0.272;
    public static final int backLSteerMotorID = 1;
    public static final int backLDriveMotorID = 8;
    public static final int backLCANcoderID = 1;

    public static final double backREncoderOffset = -0.352;
    public static final int backRSteerMotorID = 3;
    public static final int backRDriveMotorID = 2;
    public static final int backRCANcoderID = 2;

    public static final double bumpAlignKp = 0.005;
    public static final double bumpAlignKi = 0.0;
    public static final double bumpAlignKd = 0.0;
  }

  public static class VisionConstants {
    public static final int CANdleID = 1;

    public enum LightID {
      RAINBOW(0),
      POLICE(1),
      OFF(2),
      WHITE(3),
      RED(4),
      GREEN(5),
      BLUE(6);

      public final int id;

      LightID(int id) {
        this.id = id;
      }
    }
  }
  
}
