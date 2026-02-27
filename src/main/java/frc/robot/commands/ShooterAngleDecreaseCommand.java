package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShooterAngleSubsystem;

public class ShooterAngleDecreaseCommand extends Command {
    private final ShooterAngleSubsystem m_angle;

    public ShooterAngleDecreaseCommand(ShooterAngleSubsystem angle) {
        m_angle = angle;
        addRequirements(m_angle);
    }

    @Override
    public void initialize() {
        m_angle.decrease();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
