package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeRollerSubsystem;

public class IntakeRollerCommand extends Command {
    private final IntakeRollerSubsystem m_roller;
    private final double m_velocity;

    public IntakeRollerCommand(IntakeRollerSubsystem roller, double velocity) {
        m_roller = roller;
        m_velocity = velocity;
        addRequirements(m_roller);
    }

    @Override
    public void initialize() {
        m_roller.setVelocity(m_velocity);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_roller.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
