package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class TankDrive extends SubsystemBase {


  private final TalonFX leftMotor = new TalonFX(1);
  private final TalonFX rightMotor  =new TalonFX(2);
  private final TalonFX leftMotor2 = new TalonFX(3);
  private final TalonFX rightMotor2 = new TalonFX(4);


  public TankDrive() {
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();


    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;


    leftMotor.getConfigurator().apply(leftConfig);
    rightMotor.getConfigurator().apply(rightConfig);
    leftMotor2.getConfigurator().apply(leftConfig);
    rightMotor2.getConfigurator().apply(rightConfig);
   
  }


  public void arcadeDrive(double forward, double turn) {
    double leftOutput = forward + turn;
    double rightOutput = forward - turn;


    leftMotor.set(leftOutput);
    rightMotor.set(rightOutput);
    leftMotor2.set(leftOutput);
    rightMotor2.set(rightOutput);
  }


  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
    leftMotor2.stopMotor();
    rightMotor2.stopMotor();
  }
}