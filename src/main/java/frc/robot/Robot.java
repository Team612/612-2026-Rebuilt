package frc.robot;


import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot {
  //private Command m_autonomousCommand;
  Orchestra _orchestra;
  private final RobotContainer m_robotContainer;
  TalonFX[] _fxes = {new TalonFX(1, "rio"),new TalonFX(2, "rio")};
  String[] _songs = new String[] {
    "output.chrp",
    "output1.chrp",
    "output2.chrp",
    "output3.chrp",
    "output4.chrp",
    "output5.chrp",
    "output6.chrp",
    "output7.chrp",
    "output8.chrp",
    "output9.chrp",
    "output10.chrp",
    "output11.chrp",
    "output12.chrp",
    "output13.chrp",
    "output14.chrp",
    "output15.chrp",
    "output16.chrp",
    "output17.chrp",
    "output18.chrp"
  };
  int _songSelection = 0;
  int _timeToPlayLoops=0;
  Joystick _joy;
  int _lastButton=0;
  int _lastPOV=0;
  int getButton() {
    for (int i =1; i<9; i++) {
      if (_joy.getRawButton(i)) return i;
    }
    return 0;
  }
  void LoadMusicSelection(int offset) {
    _songSelection+=offset;
    if (_songSelection<0) _songSelection=_songs.length-1;
    if (_songSelection>=_songs.length) _songSelection=0;
    _orchestra.loadMusic(_songs[_songSelection]);
    System.out.println("Loading music: " + _songs[_songSelection]);
    _timeToPlayLoops=10;
  }
  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    for (int i = 0; i < _fxes.length; i++) {
      
    _orchestra.addInstrument(_fxes[i]);
    }
    _joy = new Joystick(0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // if (m_autonomousCommand != null) {
    //   CommandScheduler.getInstance().schedule(m_autonomousCommand);
    // }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    LoadMusicSelection(0);
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    // }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void teleopPeriodic() {
        /* poll gamepad */
        int btn = getButton();
        int currentPOV = _joy.getPOV();

        /* if song selection changed, auto-play it */
        if (_timeToPlayLoops > 0) {
            --_timeToPlayLoops;
            if (_timeToPlayLoops == 0) {
                /* scheduled play request */
                System.out.println("Auto-playing song.");
                _orchestra.play();
            }
        }


        /* has a button been pressed? */
        if (_lastButton != btn) {
            _lastButton = btn;

            switch (btn) {
                case 1: /* toggle play and paused */
                    if (_orchestra.isPlaying()) {
                        _orchestra.pause();
                        System.out.println("Song paused");
                    }  else {
                        _orchestra.play();
                        System.out.println("Playing song...");
                    }
                    break;
                    
                case 2:
                    if (_orchestra.isPlaying()) {
                        _orchestra.stop();
                        System.out.println("Song stopped.");
                    }  else {
                        _orchestra.play();
                        System.out.println("Playing song...");
                    }
                    break;
            }
        }

        if (_lastPOV != currentPOV) {
            _lastPOV = currentPOV;

            switch (currentPOV) {
                case 90:
                    LoadMusicSelection(+1);
                    break;
                case 270:
                    LoadMusicSelection(-1);
                    break;
            }
        }
    }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
