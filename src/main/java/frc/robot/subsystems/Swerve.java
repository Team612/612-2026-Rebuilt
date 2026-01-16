package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Swerve extends SubsystemBase {
  
  private SwerveModule frontL;
  private SwerveModule frontR;
  private SwerveModule backL;
  private SwerveModule backR;

  private Rotation2d heading;

  private Pigeon2 gyro;

  public Swerve() {
    frontL = new SwerveModule(DriveConstants.frontLDriveMotorID, DriveConstants.frontLSteerMotorID, DriveConstants.frontLCANcoderID, DriveConstants.frontLEncoderOffset);
    frontR = new SwerveModule(DriveConstants.frontRDriveMotorID, DriveConstants.frontRSteerMotorID, DriveConstants.frontRCANcoderID, DriveConstants.frontREncoderOffset);
    backL = new SwerveModule(DriveConstants.backLDriveMotorID, DriveConstants.backLSteerMotorID, DriveConstants.backLCANcoderID, DriveConstants.backLEncoderOffset);
    backR = new SwerveModule(DriveConstants.backRDriveMotorID, DriveConstants.backRSteerMotorID, DriveConstants.backRCANcoderID, DriveConstants.backREncoderOffset);

    gyro = new Pigeon2(DriveConstants.gyroID);
  }

    public void drive(ChassisSpeeds chassisSpeed){
    SwerveModuleState[] moduleStates = DriveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeed);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 1);

    frontL.setSwerveState(moduleStates[0]);
    frontR.setSwerveState(moduleStates[1]);
    backL.setSwerveState(moduleStates[2]);
    backR.setSwerveState(moduleStates[3]);
  }

  public void setPose(Pose2d pos){
    gyro.setYaw(pos.getRotation().getDegrees());
  }

  public Rotation2d getHeading(){
    return heading;
  }

  @Override
  public void periodic() {
    heading = Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360));
  }
}
