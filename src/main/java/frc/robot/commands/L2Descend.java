package frc.robot.commands;

import frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj2.command.Command;


public class L2Descend extends Command {
    Climb m_climb;

    public L2Descend(Climb m_climb) {
        this.m_climb = m_climb;
        addRequirements(m_climb);
    }

    @Override
    public void initialize() {
        m_climb.SetMotor(-0.5);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        m_climb.SetMotor(0);
    }

    @Override
    public boolean isFinished() {
        if(m_climb.getMotorPosition() <= 0) {
            return true;
        }
        return false;
    }
}
