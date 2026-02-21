package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Transfer extends SubsystemBase{
    // SparkMax motor1 is 550 motor, SparkFlex motors are vortex motors
    private final SparkFlex hopperTop = new SparkFlex(2, MotorType.kBrushless);
    private final SparkFlex hopperBottom = new SparkFlex(3, MotorType.kBrushless);
    private final SparkFlex feed = new SparkFlex(4, MotorType.kBrushless);

    public Transfer() {}

    public void setHopperTop(double speed){
        hopperTop.set(speed);
    }
    
    public void setHopperBottom(double speed){
        hopperBottom.set(speed);
    }
    
    public void setFeed(double speed){
        feed.set(speed);
    }
    

    @Override
    public void periodic() {

    }
}