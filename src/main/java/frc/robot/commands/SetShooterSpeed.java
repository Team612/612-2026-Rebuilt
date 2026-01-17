// src/main/java/frc/robot/commands/NeoSetSpeedCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

public class SetShooterSpeed extends InstantCommand {

    public SetShooterSpeed(Shooter neoSubsystem, double speed) {
        super(() -> neoSubsystem.setShooterSpeed(speed), neoSubsystem);
    }
}
