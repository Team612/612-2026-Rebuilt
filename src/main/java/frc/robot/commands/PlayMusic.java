package frc.robot.commands;

import frc.robot.subsystems.MusicPlayer;
import edu.wpi.first.wpilibj2.command.Command;

public class PlayMusic extends Command {

    private final MusicPlayer m;

    public PlayMusic(MusicPlayer m_m) {
        m = m_m;
        addRequirements(m);
    }

    // Called once when the command is first scheduled — start playback here
    @Override
    public void initialize() {
        m.playMusic();
    }

    // execute() intentionally left empty — Orchestra handles playback internally
    @Override
    public void execute() {}

    // Stop music when command ends or is interrupted
    @Override
    public void end(boolean interrupted) {
        m.stopMusic();
    }

    // Command runs until interrupted (e.g. button released)
    @Override
    public boolean isFinished() {
        return false;
    }
}