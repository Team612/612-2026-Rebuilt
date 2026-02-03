package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransferConstants;

public class Transfer extends SubsystemBase{
    // SparkMax motor1 is 550 motor, SparkFlex motors are vortex motors
    private final SparkMax motor1 = new SparkMax(TransferConstants.motor1ID, MotorType.kBrushless);
    private final SparkFlex motor2 = new SparkFlex(TransferConstants.motor2ID, MotorType.kBrushless);
    private final SparkFlex motor3 = new SparkFlex(TransferConstants.motor3ID, MotorType.kBrushless);
    private final SparkFlex motor4 = new SparkFlex(TransferConstants.motor4ID, MotorType.kBrushless);

    public Transfer() {}

    public void setMotor1(double speed){
        motor1.set(speed);
    }
    
    public void setMotor2(double speed){
        motor2.set(speed);
    }
    
    public void setMotor3(double speed){
        motor3.set(speed);
    }
    
    public void setMotor4(double speed){
        motor4.set(speed);
    }

    @Override
    public void periodic() {

    }
}
