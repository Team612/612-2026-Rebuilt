package frc.robot;

public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {

    public static final int gyroID = 0;

    public static final double DEADBAND = 0.05;

    public static final double kCountsPerRevolution = 1440.0;
    public static final double kWheelDiameterMeter = 0.07;
    public static final double encoderToMeters = ((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
  }

  public static class IntakeConstants{
    public static final int INTAKE_MOTOR_ID = 10; // this is a guess

    public static final double INTAKE_SPEED = 0.85;  // this is a guess
    public static final double OUTTAKE_SPEED = -0.65;  // this is a guess
  }
  
}