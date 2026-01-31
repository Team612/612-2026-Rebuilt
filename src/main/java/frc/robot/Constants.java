package frc.robot;

public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    public static final int gyroID = 0;
    public static final double DEADBAND = 0.05;
    public static final double kWheelDiameterMeters = 0.07;
    public static final double kCountsPerRevolution = 1440.0;
    public static final double encoderDistancePerPulse = (Math.PI * kWheelDiameterMeters) / kCountsPerRevolution;
    public static final double metersToInches = 39.3700787402;
  }
  
}