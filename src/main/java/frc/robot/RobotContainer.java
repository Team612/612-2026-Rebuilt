package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.PlayMusic;
import frc.robot.subsystems.MusicPlayer;

public class RobotContainer {

    // Subsystems
    private final MusicPlayer musicPlayer;

    // Controller
    private final CommandXboxController driverController = new CommandXboxController(0);

    public RobotContainer() {
        musicPlayer = new MusicPlayer(1);
        configureBindings();
    }

    private void configureBindings() {
        // Hold A to play music, releases stop it
        driverController.a().whileTrue(new PlayMusic(musicPlayer));

        // Press B to manually stop music
        driverController.b().onTrue(new InstantCommand(musicPlayer::stopMusic, musicPlayer));
    }

    public Command getAutonomousCommand() {
        return null;
    }

    public MusicPlayer getMusicPlayer() {
        return musicPlayer;
    }
}