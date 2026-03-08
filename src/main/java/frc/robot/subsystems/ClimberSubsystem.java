package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    // --- CONSTANTS ---
    private static final int kClimberMotorId = 12;
    
    // POSITIONS (In Rotations of the motor shaft)
    // You MUST tune these. 0 is usually fully retracted (robot up), 
    // and positive is extended (hook up).
    public static final double kPositionDown = 0.0; 
    public static final double kPositionUp = 745.0; // Example: 80 rotations to reach the chain
    public double targetPos = 0;
    
    // SAFETY LIMITS
    private static final double kSoftLimitReverse = -1.0;
    private static final double kSoftLimitForward = 755.0; // Slightly past max height

    // --- HARDWARE ---
    private final TalonFX m_motor = new TalonFX(kClimberMotorId);

    // --- CONTROL REQUESTS ---
    // DutyCycleOut = Manual % Output
    private final DutyCycleOut m_manualControl = new DutyCycleOut(0);
    
    // PositionVoltage = PID Position Control (Go to X rotations)
    // EnableFOC=true is usually smoother/stronger if you have Pro license, otherwise false
    private final PositionVoltage m_positionControl = new PositionVoltage(0).withSlot(0).withEnableFOC(false);

    public ClimberSubsystem() {
        configureMotor();
    }


    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // 1. Motor Safety & Direction
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Check this!
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // CRITICAL for climbers

        // 2. PID Settings (Slot 0)
        // Climbers need a high P to hold position against gravity
        config.Slot0.kP = 2.0; // Start here. If it oscillates, lower it. If it sags, raise it.
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.05; // Small D helps stop oscillation
        config.Slot0.kV = 0.12; // Feedforward for speed

        // 3. Current Limits (Prevent burning out if jammed)
        config.CurrentLimits.StatorCurrentLimit = 80; // High torque allowed briefly
        config.CurrentLimits.SupplyCurrentLimit = 40; // Breaker safety
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // 4. Soft Limits (Prevents breaking the mechanism)
        // ENABLE THESE after you have manually verified the direction!
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kSoftLimitForward;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kSoftLimitReverse;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        m_motor.getConfigurator().apply(config);
        
        // Reset encoder to 0 on boot (Assuming robot starts with climber down)
        m_motor.setPosition(0);
    }

    // ==========================================
    // ACTION METHODS
    // ==========================================

    /**
     * Manual control (e.g., Joystick)
     * @param percentOutput -1.0 to 1.0
     */
    public void setPower(double percentOutput) {
        m_motor.setControl(m_manualControl.withOutput(percentOutput));
    }

    /**
     * Go to a specific rotation count using PID
     */
    public void setPosition(double rotations) {
        m_motor.setControl(m_positionControl.withPosition(rotations));
    }

    /**
     * Stop the motor
     */
    public void stop() {
        m_motor.setControl(m_manualControl.withOutput(0));
    }

    /**
     * Zero the encoder (Call this if manual calibration is needed)
     */
    public void resetEncoder() {
        m_motor.setPosition(0);
    }

    // ==========================================
    // TELEMETRY
    // ==========================================

    public double getPosition() {
        return m_motor.getPosition().getValueAsDouble();
    }

    public boolean atSetpoint(double target, double tolerance) {
        return Math.abs(getPosition() - target) < tolerance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber/Pos", getPosition());
        SmartDashboard.putNumber("Climber/Amps", m_motor.getStatorCurrent().getValueAsDouble());
    }
}