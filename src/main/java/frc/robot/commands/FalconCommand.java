package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.FalconSubsystem;

public class FalconCommand extends Command {
    private final FalconSubsystem m_falcon;
    private final double m_velocity;

    public FalconCommand(FalconSubsystem falcon, double velocity) {
        m_falcon = falcon;
        m_velocity = velocity;
        addRequirements(m_falcon);
    }

    @Override
    public void initialize() {
        m_falcon.setVelocity(m_velocity);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_falcon.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
