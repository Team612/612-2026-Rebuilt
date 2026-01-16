package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;

public class SwerveModule {
    private TalonFX driveMotor;
    private TalonFX steerMotor;
    public CANcoder angleEncoder;
    private double encoderOffset;
    private TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    private TalonFXConfiguration steerConfig = new TalonFXConfiguration();
    private PIDController turnPIDController = new PIDController(DriveConstants.kp, 0, 0.0);

    public SwerveModule(int drivingMotorID, int steerMotorID, int angleEncoderID, double encoderOffset){
      driveMotor = new TalonFX(drivingMotorID);
      steerMotor = new TalonFX(steerMotorID);
      angleEncoder = new CANcoder(angleEncoderID);
      this.encoderOffset = encoderOffset;

      driveMotor.setNeutralMode(NeutralModeValue.Brake);
      steerMotor.setNeutralMode(NeutralModeValue.Brake);

      driveConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.driveSupplyCurrent;
      driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

      driveConfig.CurrentLimits.StatorCurrentLimitEnable = false;

      steerConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.steerSupplyCurrent;
      steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

      steerConfig.CurrentLimits.StatorCurrentLimitEnable = false;

      driveMotor.getConfigurator().apply(driveConfig);
      steerMotor.getConfigurator().apply(steerConfig);
      driveMotor.setNeutralMode(NeutralModeValue.Brake);
      steerMotor.setNeutralMode(NeutralModeValue.Brake);
      turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setSwerveState(SwerveModuleState desiredState){
      desiredState.optimize(new Rotation2d(getCurrentAngle()));

      driveMotor.set(desiredState.speedMetersPerSecond);

      steerMotor.set(-turnPIDController.calculate(getCurrentAngle(), desiredState.angle.getRadians()));
    }

    public double getCurrentVelocity(){
      return driveMotor.getVelocity().getValueAsDouble() * DriveConstants.rotationsToMeters;
    }

    public SwerveModulePosition getCurrentWheelPosition(){
      return new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble() * DriveConstants.rotationsToMeters, new Rotation2d(getCurrentAngle()));
    }

    public double getCurrentAngle() {
      double rotations = angleEncoder.getAbsolutePosition().getValueAsDouble() - encoderOffset;
      if (rotations < -0.5)
        rotations += 1;
      return rotations * 2 * Math.PI;
    }
}
