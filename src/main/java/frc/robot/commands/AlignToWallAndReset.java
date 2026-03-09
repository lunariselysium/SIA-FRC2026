package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToWallAndReset extends Command {
    // Macros for sensor selection
    public static final int BACK_SENSOR = 1;
    public static final int RIGHT_SENSOR = 2;

    private final CommandSwerveDrivetrain m_drivetrain;
    private final int m_sensorId;
    private final double m_targetSensorDistance;
    private final Double m_overrideX;
    private final Double m_overrideY;
    private boolean hasReachedTarget;

    // Tuning variables
    private final double TOLERANCE_METERS = 0.03; // Stop when within 3cm of target
    private final double MAX_SPEED = 0.8; // Max meters per second
    
    // PID Constants
    private final double kP = 3.0; 
    private final double kI = 0.05; 
    private final double kD = 0.0; 
    
    private final PIDController m_pidController;

    /**
     * @param sensorId Which sensor to use (BACK_SENSOR or RIGHT_SENSOR).
     * @param targetSensorDistance The distance the CANrange should read when finished.
     * @param overrideX The exact X coordinate to snap to (or null to keep current X).
     * @param overrideY The exact Y coordinate to snap to (or null to keep current Y).
     */
    public AlignToWallAndReset(CommandSwerveDrivetrain drivetrain, int sensorId, double targetSensorDistance, Double overrideX, Double overrideY) {
        m_drivetrain = drivetrain;
        m_sensorId = sensorId;
        m_targetSensorDistance = targetSensorDistance;
        m_overrideX = overrideX;
        m_overrideY = overrideY;
        addRequirements(drivetrain);

        // Initialize the WPILib PID Controller
        m_pidController = new PIDController(kP, kI, kD);
        m_pidController.setTolerance(TOLERANCE_METERS);
    }

    @Override
    public void initialize() {
        hasReachedTarget = false;
        
        // Reset the PID controller to clear out accumulated I-term/D-term from previous runs
        m_pidController.reset();
    }

    // Helper method to grab the correct sensor distance
    private double getCurrentDistance() {
        if (m_sensorId == RIGHT_SENSOR) {
            System.out.println("***********" + m_drivetrain.getRightSensorDistanceMeters());
            return m_drivetrain.getRightSensorDistanceMeters();
        } else if (m_sensorId == BACK_SENSOR){
            return m_drivetrain.getRearSensorDistanceMeters();
        }
        return 999999;
    }

    @Override
    public void execute() {
        double currentDistance = getCurrentDistance();

        // Calculate speed using WPILib's PID control.
        // It outputs the necessary speed to reach the setpoint, natively handling the math 
        // needed to reverse direction if you overshoot.
        double speed = m_pidController.calculate(currentDistance, m_targetSensorDistance);

        // Check if we are within the tolerance we set in the constructor
        if (m_pidController.atSetpoint()) {
            hasReachedTarget = true;
            return; 
        }

        speed = MathUtil.clamp(speed, -MAX_SPEED, MAX_SPEED);

        // Apply speed to the correct axis based on the sensor chosen
        if (m_sensorId == RIGHT_SENSOR) {
            m_drivetrain.driveRawRobotRelative(0.0, speed, 0.0);
        } else {
            m_drivetrain.driveRawRobotRelative(speed, 0.0, 0.0);
        }
    }

    @Override
    public boolean isFinished() {
        return hasReachedTarget;
    }

    @Override
    public void end(boolean interrupted) {
        // 1. Stop the robot
        m_drivetrain.driveRawRobotRelative(0.0, 0.0, 0.0);

        // 2. Apply the specific overrides if we succeeded
        if (!interrupted) {
            Pose2d currentPose = m_drivetrain.getState().Pose;
            
            // If the user passed null, use the odometry's current guess.
            // If the user passed a number, use that exact number!
            double finalX = (m_overrideX != null) ? m_overrideX : currentPose.getX();
            double finalY = (m_overrideY != null) ? m_overrideY : currentPose.getY();

            // Check which alliance we are currently on
            Optional<Alliance> alliance = DriverStation.getAlliance();
            boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;

            final double FIELD_LENGTH = 16.541;
            final double FIELD_WIDTH = 8.07;

            if (m_overrideX != null) {
                // If Red, mirror X across the center of the field
                finalX = isRed ? (FIELD_LENGTH - m_overrideX) : m_overrideX;
            }
            if (m_overrideY != null) {
                // If Red, mirror Y across the center of the field
                finalY = isRed ? (FIELD_WIDTH - m_overrideY) : m_overrideY;
            }

            // Reset the pose
            m_drivetrain.resetPose(new Pose2d(finalX, finalY, currentPose.getRotation()));
            System.out.println("Snapped to Pose: X=" + finalX + " Y=" + finalY);
        }
    }
}