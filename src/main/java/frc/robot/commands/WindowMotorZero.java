// src/main/java/frc/robot/commands/WindowMotorZero.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class WindowMotorZero extends Command {

    private static final double HOMING_SPEED = 0.2; // adjust sign/direction!

    private final Shooter windowMotor;

    public WindowMotorZero(Shooter subsystem) {
        this.windowMotor = subsystem;
        addRequirements(windowMotor);
    }

    @Override
    public void initialize() {
        // Optional: clear encoder before homing
        // windowMotor.resetEncoder();
    }

    @Override
    public void execute() {
        // Drive toward the limit switch
        windowMotor.setShooterSpeed(HOMING_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop and make sure encoder is zeroed
        windowMotor.stopShooterMotor();
        windowMotor.resetTiltEncoder();
    }

    @Override
    public boolean isFinished() {
        // Finish when limit switch is hit
        return windowMotor.isLimitSwitchPressed();
    }
}
