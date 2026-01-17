package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.MusicPlayer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is used,
 * subsystems, commands, and button mappings are defined here.
 */
public class RobotContainer {

    // Subsystems
    private final MusicPlayer musicPlayer;

    // Example controller (optional, for triggering music play)
    private final CommandXboxController driverController = new CommandXboxController(0);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // Initialize subsystems
        musicPlayer = new MusicPlayer(1); // ID 1 for your TalonFX

        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your button->command mappings.  
     * Currently, example: press A to play music.
     */
    private void configureBindings() {
        // Press 'A' button to play music
        driverController.a().whileTrue(
            new edu.wpi.first.wpilibj2.command.InstantCommand(() -> musicPlayer.playMusic())
        );
    }

    /**
     * Use this to pass the autonomous command to the main Robot class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Currently no autonomous command; return null or your autonomous command here
        return null;
    }

    /** Getter for MusicPlayer subsystem */
    public MusicPlayer getMusicPlayer() {
        return musicPlayer;
    }
}
