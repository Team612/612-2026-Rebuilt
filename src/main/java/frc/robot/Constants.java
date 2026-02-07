package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    public static final int leftMotorID = 1; // this is a guess
    public static final int rightMotorID = 3; // this is a guess
    public static final int leftMotor2ID = 2; // this is a guess
    public static final int rightMotor2ID = 4; // this is a guess


    public static final int gyroID = 0;


    public static final double DEADBAND = 0.05;


    public static final double kCountsPerRevolution = 1440.0;
    public static final double kWheelDiameterMeter = 0.192;
    public static final double encoderToMeters = ((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);


    public static final double trackWidth = 0.508;
    public static final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(trackWidth);


    public static final double maxAttainableSpeed = 5.0; 
  }

  public static class IntakeConstants{
    public static final int INTAKE_MOTOR_ID = 10; // this is a guess

    public static final double INTAKE_SPEED = 0.85;  // this is a guess
    public static final double OUTTAKE_SPEED = -0.65;  // this is a guess
  }
  
}