package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MusicPlayer extends SubsystemBase {

    private TalonFX driveMotor;
    private Orchestra orch;

    public MusicPlayer(int drivingMotorID) {
        // Step 1: create the motor
        driveMotor = new TalonFX(drivingMotorID);

        // Step 2: initialize the Orchestra
        orch = new Orchestra();

        // Step 3: add instruments
        orch.addInstrument(driveMotor);

        // Step 4: load the music file
        orch.loadMusic("output.chrp"); // must be in deploy folder
    }

    public void playMusic() {
        if (orch != null) {
            orch.play();
        }
    }

    @Override
    public void periodic() {
        // Called once per scheduler run
    }
}
