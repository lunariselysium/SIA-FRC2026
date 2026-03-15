package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    
    // --- SETPOINTS (IN ROTATIONS) ---
    private static final double kHoodPositionLow = 0.0;  
    private static final double kHoodPositionHigh = 5.5; 

    public enum HoodPosition {
        LOW,
        HIGH
    }

    // --- MOTOR CONSTANTS ---
    private static final int kLeftFlywheelId = 9;
    private static final int kRightFlywheelId = 10;
    
    private static final int kFeederKrakenId = 16;
    private static final int kShooterKrakenId = 11; // this is the 'Indexer' right before flywheels
    
    private static final int kHoodMotorId = 15;

    // --- HARDWARE ---
    private final TalonFX m_leftFlywheel = new TalonFX(kLeftFlywheelId);
    private final TalonFX m_rightFlywheel = new TalonFX(kRightFlywheelId);
    
    private final TalonFX m_feederMotor = new TalonFX(kFeederKrakenId);
    private final TalonFX m_indexerMotor = new TalonFX(kShooterKrakenId);
    
    private final TalonFX m_hoodMotor = new TalonFX(kHoodMotorId);

    // --- CONTROLLERS ---
    private final VelocityVoltage m_flywheelControl
         = new VelocityVoltage(0).withEnableFOC(true).withSlot(0);
    private final VelocityVoltage m_feederControl = new VelocityVoltage(0).withSlot(0);
    
    // Position control request for the Hood
    private final PositionVoltage m_hoodControl = new PositionVoltage(0).withSlot(0);

    // --- STATE VARIABLES ---
    private double m_targetFlywheelVelocity = 0;
    private double m_targetHoodPosition = kHoodPositionLow;
    private HoodPosition m_currentHoodState = HoodPosition.LOW;

    public ShooterSubsystem() {
        configureFlywheels();
        configureFeeders();
        configureHood();
    }

    // ==========================================
    // CONFIGURATION METHODS
    // ==========================================
    
    private void configureFlywheels() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kS = 0.5;
        config.Slot0.kV = 0.108;
        config.Slot0.kP = 0.4;
        config.Slot0.kI = 0.05;
        config.Slot0.kD = 0.0;

        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;

        config.CurrentLimits.StatorCurrentLimit = 100;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        
        config.CurrentLimits.SupplyCurrentLimit = 50; 
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        applyConfig(m_leftFlywheel, config, "Left Flywheel");

        m_rightFlywheel.setControl(new Follower(kLeftFlywheelId, MotorAlignmentValue.Opposed));

        m_leftFlywheel.setNeutralMode(NeutralModeValue.Coast);
        m_rightFlywheel.setNeutralMode(NeutralModeValue.Coast);
    }

    private void configureFeeders() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.7;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        applyConfig(m_feederMotor, config, "Feeder Kraken");
        applyConfig(m_indexerMotor, config, "Indexer Kraken");

        m_feederMotor.setNeutralMode(NeutralModeValue.Coast);
        m_indexerMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    private void configureHood() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // --- HOOD PID TUNING ---
        config.Slot0.kP = 1.2; 
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kS = 0.0; // Static friction feedforward if needed

        config.Voltage.PeakForwardVoltage = 8;
        config.Voltage.PeakReverseVoltage = -8;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        applyConfig(m_hoodMotor, config, "Hood Motor");

        m_hoodMotor.setNeutralMode(NeutralModeValue.Brake);
        
        // IMPORTANT: Assuming the hood is at the "0" (LOW) position when the robot turns on.
        // If it's not, you'll need a way to zero it (e.g., hard stop zeroing sequence or a limit switch).
        m_hoodMotor.setPosition(0); 
    }

    private void applyConfig(TalonFX motor, TalonFXConfiguration config, String name) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs to " + name + ", error code: " + status.toString());
        }
    }

    // ==========================================
    // ACTION METHODS (For Commands to use)
    // ==========================================

    /**
     * Spins up the flywheels (The Falcons).
     * @param velocity Rotations per second (RPS) or RPM depending on Phoenix configs
     */
    public void setFlywheelVelocity(double velocity) {
        m_targetFlywheelVelocity = velocity;
        m_leftFlywheel.setControl(m_flywheelControl.withVelocity(velocity));
    }

    /**
     * Sets the hood to one of the predefined states (LOW or HIGH).
     */
    public void setHoodState(HoodPosition position) {
        m_currentHoodState = position;
        
        if (position == HoodPosition.LOW) {
            m_targetHoodPosition = kHoodPositionLow;
        } else {
            m_targetHoodPosition = kHoodPositionHigh;
        }

        // Send the position request to the motor controller
        m_hoodMotor.setControl(m_hoodControl.withPosition(m_targetHoodPosition));
    }

    /**
     * Runs the feeder and indexer wheels to push the note/piece into the flywheels.
     * @param velocity Speed to run the feeders
     */
    public void runFeeder(double velocity) {
        m_feederMotor.setControl(m_feederControl.withVelocity(velocity));
        m_indexerMotor.setControl(m_feederControl.withVelocity(velocity));
    }

    /**
     * Stops the feeder wheels completely.
     */
    public void stopFeeder() {
        m_feederMotor.setControl(new DutyCycleOut(0));
        m_indexerMotor.setControl(new DutyCycleOut(0));
    }

    /**
     * Stops everything in the shooter subsystem.
     */
    public void stopEverything() {
        setFlywheelVelocity(0);
        stopFeeder();
        m_hoodMotor.setControl(new DutyCycleOut(0));
    }

    // ==========================================
    // TELEMETRY & FEEDBACK
    // ==========================================

    public double getFlywheelVelocity() {
        return m_leftFlywheel.getVelocity().getValueAsDouble();
    }

    public double getTargetFlywheelVelocity() {
        return m_targetFlywheelVelocity;
    }

    public double getFeederVelocity() {
        return m_feederMotor.getVelocity().getValueAsDouble();
    }

    /**
     * Gets the current hood position in rotations.
     */
    public double getHoodPosition() {
        return m_hoodMotor.getPosition().getValueAsDouble();
    }
    
    public boolean isFlywheelAtSpeed(double tolerance) {
        return Math.abs(getFlywheelVelocity() - m_targetFlywheelVelocity) <= tolerance;
    }

    /**
     * Checks if the hood is at the requested position.
     * @param toleranceRotations acceptable error in motor rotations
     */
    public boolean isHoodAtPosition(double toleranceRotations) {
        return Math.abs(getHoodPosition() - m_targetHoodPosition) <= toleranceRotations;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Flywheel Vel", getFlywheelVelocity());
        SmartDashboard.putNumber("Shooter/Flywheel Target", m_targetFlywheelVelocity);
        
        SmartDashboard.putNumber("Shooter/Hood Position (rots)", getHoodPosition());
        SmartDashboard.putNumber("Shooter/Hood Target (rots)", m_targetHoodPosition);
        SmartDashboard.putString("Shooter/Hood State", m_currentHoodState.name());

        SmartDashboard.putNumber("Feeder Vel", getFeederVelocity());
    }
}