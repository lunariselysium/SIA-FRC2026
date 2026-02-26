package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final double m_falconVelocity;
    private final double m_krakenVelocity;

    public ShooterCommand(ShooterSubsystem shooter, double falconVelocity, double krakenVelocity) {
        m_shooter = shooter;
        m_falconVelocity = falconVelocity;
        m_krakenVelocity = krakenVelocity;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_shooter.setShooter(m_falconVelocity, m_krakenVelocity);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_shooter.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
