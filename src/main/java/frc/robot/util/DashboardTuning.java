package frc.robot.util;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TransferConstants;

public final class DashboardTuning {
  private DashboardTuning() {}

  public static void init() {
    SmartDashboard.putNumber("Hopper 1", TransferConstants.hopperTopSpeed);
    SmartDashboard.putNumber("Hopper 2", TransferConstants.hopperBottomSpeed);
    SmartDashboard.putNumber("FeedVolts", TransferConstants.feedVolts);

    SmartDashboard.putNumber("Intake Ball 1", IntakeConstants.upperIntakeSpeed);
    SmartDashboard.putNumber("Intake Ball 2", IntakeConstants.lowerIntakeSpeed);
    SmartDashboard.putNumber("AgitatorFeed", IntakeConstants.jammerFeedSpeed);
    SmartDashboard.putNumber("AgitatorIntake", IntakeConstants.jammerIntakeSpeed);

    SmartDashboard.putNumber("Shooter RPM", ShooterConstants.defaultShootRPM);
    SmartDashboard.putNumber("RPM window", 100);

    ShuffleboardTab tab = Shuffleboard.getTab("Tuning");

    tab.addPersistent("Hopper 1", TransferConstants.hopperTopSpeed)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", -1.0, "max", 1.0));
    tab.addPersistent("Hopper 2", TransferConstants.hopperBottomSpeed)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", -1.0, "max", 1.0));
    tab.addPersistent("FeedVolts", TransferConstants.feedVolts)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0.0, "max", 12.0));

    tab.addPersistent("AgitatorFeed", IntakeConstants.jammerFeedSpeed)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", -1.0, "max", 1.0));
    tab.addPersistent("AgitatorIntake", IntakeConstants.jammerIntakeSpeed)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", -1.0, "max", 1.0));

    tab.addPersistent("Shooter RPM", ShooterConstants.defaultShootRPM)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0.0, "max", 6000.0));
  }
}

