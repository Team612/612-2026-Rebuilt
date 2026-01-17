// src/main/java/frc/robot/commands/NeoStopCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

public class StopShooter extends InstantCommand {

    public StopShooter(Shooter neoSubsystem) {
        super(() -> neoSubsystem.stopShooterMotor(), neoSubsystem);
    }
}
