package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransferConstants;

public class Transfer extends SubsystemBase{
  private final SparkFlex hopperTop = new SparkFlex(TransferConstants.hopperTopID, MotorType.kBrushless);
  private final SparkFlex hopperBottom = new SparkFlex(TransferConstants.hopperBottomID, MotorType.kBrushless);
  private final SparkFlex feed = new SparkFlex(TransferConstants.feedID, MotorType.kBrushless);

  public Transfer() {
    SparkBaseConfig feedConfig = new SparkFlexConfig();
    SparkBaseConfig hopperTopConfig = new SparkFlexConfig();
    SparkBaseConfig hopperBottomConfig = new SparkFlexConfig();

    feedConfig.idleMode(IdleMode.kBrake).inverted(false);
    hopperTopConfig.idleMode(IdleMode.kBrake).inverted(false);
    hopperBottomConfig.idleMode(IdleMode.kBrake).inverted(false);

    feed.configure(feedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    hopperTop.configure(hopperTopConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    hopperBottom.configure(hopperBottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

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
    SmartDashboard.putNumber("hopperTopGet",hopperTop.get());
    SmartDashboard.putNumber("hopperBottomGet",hopperBottom.get());
    SmartDashboard.putNumber("feedGet",feed.get());
  }
}
