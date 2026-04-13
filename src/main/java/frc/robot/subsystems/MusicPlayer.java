package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        // Step 4: load the music file and check status
        StatusCode status = orch.loadMusic("musiclab.chrp"); // must be in deploy folder
        if (!status.isOK()) {
            System.err.println("[MusicPlayer] Failed to load music file: " + status.toString());
        }
    }

    public void playMusic() {
        if (orch != null) {
            orch.play();
        }
    }

    public void stopMusic() {
        if (orch != null) {
            orch.stop();
        }
    }

    public boolean isPlaying() {
        return orch != null && orch.isPlaying();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Music Playing", isPlaying());
    }
}