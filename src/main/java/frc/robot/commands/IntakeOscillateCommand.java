package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOscillateCommand extends Command {
    private final IntakeSubsystem m_intake;
    private boolean movingDown = true;

    // How close to the target do we need to be before switching direction?
    private static final double TOLERANCE = 0.5; 

    public IntakeOscillateCommand(IntakeSubsystem intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        // m_intake.runRollers(); // Keep rollers running while shaking
        movingDown = true;
        m_intake.pivotDown();
    }

    @Override
    public void execute() {
        double currentPos = m_intake.getPivotPosition();

        if (movingDown) {
            // We are moving towards DOWN. Check if we are there.
            if (Math.abs(currentPos - m_intake.m_posDown) < TOLERANCE) {
                // We reached the bottom, switch to Partial Retract
                movingDown = false;
                m_intake.pivotOscillate();
            } else {
                // Ensure we are commanding down
                m_intake.pivotDown();
            }
        } else {
            // We are moving towards PARTIAL UP (Oscillate). Check if we are there.
            if (Math.abs(currentPos - m_intake.m_posOscillate) < TOLERANCE) {
                // We reached partial up, switch back to Down
                movingDown = true;
                m_intake.pivotDown();
            } else {
                // Ensure we are commanding oscillate
                m_intake.pivotOscillate();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.pivotDown();
        // m_intake.stopRollers();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}