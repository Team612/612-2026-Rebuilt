// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {

  private CANdle candle = new CANdle(VisionConstants.CANdleID);

  private final RainbowAnimation rainbow = new RainbowAnimation(1.0, 0.6, 64);

  private final StrobeAnimation red = new StrobeAnimation(255, 0, 0, 0, 0.02, 64);
  private final StrobeAnimation blue = new StrobeAnimation(0, 0, 255, 0, 0.02, 64);

  private boolean police = false;

  private double timer = 0;

  public Vision() {}

  public void setLightID(int id){
    candle.setLEDs(0,0,0);
    candle.clearAnimation(0);

    if (id == 1)
      police = true;
    else
      police = false;

    if (id == 0)
      candle.animate(rainbow);
    if (id == 2)
      candle.setLEDs(0,0,0);
    if (id == 3)
      candle.setLEDs(255,255,255);
    if (id == 4)
      candle.setLEDs(255,0,0);
    if (id == 5)
      candle.setLEDs(0,255,0);
    if (id == 6)
      candle.setLEDs(0,0,255);
  }

  @Override
  public void periodic() {
    if (police){
      timer++;
      if (timer < 25)
        candle.animate(red);
      else
        candle.animate(blue);

      if (timer == 50)
        timer = 0;
    }
  }
}
