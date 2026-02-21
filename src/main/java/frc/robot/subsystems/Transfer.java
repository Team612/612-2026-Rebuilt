package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransferConstants;

public class Transfer extends SubsystemBase{
    // SparkMax motor1 is 550 motor, SparkFlex motors are vortex motors
    private final SparkFlex hopperTop = new SparkFlex(TransferConstants.hopperTopID, MotorType.kBrushless);
    private final SparkFlex hopperBottom = new SparkFlex(TransferConstants.hopperBottomID, MotorType.kBrushless);
    private final SparkFlex feed = new SparkFlex(TransferConstants.feedID, MotorType.kBrushless);

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
        SmartDashboard.putNumber("hopperTopSpeed",hopperTop.get());
        SmartDashboard.putNumber("hopperBottomSpeed",hopperBottom.get());
        SmartDashboard.putNumber("feedSpeed",feed.get());
    }
}