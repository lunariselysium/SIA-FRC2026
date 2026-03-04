package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    
    // --- MOTOR CONSTANTS ---
    private static final int kLeftFlywheelId = 9;
    private static final int kRightFlywheelId = 10;
    
    private static final int kFeederKrakenId = 16;
    private static final int kShooterKrakenId = 11; // this is the 'Indexer' right before flywheels
    
    private static final int kHoodMotorId = 15;
    private static final int kHoodCanRangeId = 0;

    // --- HARDWARE ---
    private final TalonFX m_leftFlywheel = new TalonFX(kLeftFlywheelId);
    private final TalonFX m_rightFlywheel = new TalonFX(kRightFlywheelId);
    
    private final TalonFX m_feederMotor = new TalonFX(kFeederKrakenId);
    private final TalonFX m_indexerMotor = new TalonFX(kShooterKrakenId);
    
    private final TalonFX m_hoodMotor = new TalonFX(kHoodMotorId);
    private final CANrange m_hoodRange = new CANrange(kHoodCanRangeId);

    // --- CONTROLLERS ---
    private final VelocityVoltage m_flywheelControl = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage m_feederControl = new VelocityVoltage(0).withSlot(0);
    private final PositionVoltage m_hoodControl = new PositionVoltage(0).withSlot(0);

    private final edu.wpi.first.math.controller.PIDController m_hoodPID = 
        new edu.wpi.first.math.controller.PIDController(0.05, 0, 0);

    // --- STATE VARIABLES ---
    private double m_targetFlywheelVelocity = 0;
    private double m_targetHoodPosition = 0; // motor encoder position
    private double m_targetHoodDistanceMm = 100; // CANrange data
    
    // Distance Sensor Filter logic
    private static final double kMmPerMeter = 1000.0;
    private static final double kAlpha = 0.05;
    private double m_filteredHoodDistance = 0;
    private boolean m_firstReading = true;

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
        config.Slot0.kP = 0.6;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;

        applyConfig(m_leftFlywheel, config, "Left Flywheel");

        // Set Right to follow Left, but spinning the opposite direction
        m_rightFlywheel.setControl(new Follower(kLeftFlywheelId, MotorAlignmentValue.Opposed));

        m_leftFlywheel.setNeutralMode(NeutralModeValue.Coast);
        m_rightFlywheel.setNeutralMode(NeutralModeValue.Coast);
    }

    private void configureFeeders() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.6;
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
        config.Slot0.kP = 1;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0.1;
        config.Voltage.PeakForwardVoltage = 8;
        config.Voltage.PeakReverseVoltage = -8;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        applyConfig(m_hoodMotor, config, "Hood Motor");

        m_hoodMotor.setNeutralMode(NeutralModeValue.Brake);
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
     * Sets the target motor encoder position for the Hood.
     */
    public void setHoodPosition(double position) {
        m_targetHoodPosition = position;
        m_hoodMotor.setControl(m_hoodControl.withPosition(position));
    }

    /**
     * Sets the target CANrange position for hood
     * @param distanceMm
     */
    public void setHoodDistanceMm(double distanceMm) {
        m_targetHoodDistanceMm = distanceMm;
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

    public double getFeederVelocity() {
        return m_feederMotor.getVelocity().getValueAsDouble();
    }

    public double getHoodPosition() {
        return m_hoodMotor.getPosition().getValueAsDouble();
    }

    public double getFilteredHoodDistance() {
        double rawDistance = m_hoodRange.getDistance().getValueAsDouble() * kMmPerMeter;
        
        if (m_firstReading) {
            m_filteredHoodDistance = rawDistance;
            m_firstReading = false;
        } else {
            m_filteredHoodDistance = kAlpha * rawDistance + (1 - kAlpha) * m_filteredHoodDistance;
        }
        
        return m_filteredHoodDistance;
    }
    
    /**
     * Checks if the flywheels are spun up to the target speed.
     */
    public boolean isFlywheelAtSpeed(double tolerance) {
        return Math.abs(getFlywheelVelocity() - m_targetFlywheelVelocity) <= tolerance;
    }

    /**
     * Checks if the hood is at the requested position.
     */
    public boolean isHoodAtPosition(double tolerance) {
        return Math.abs(getHoodPosition() - m_targetHoodPosition) <= tolerance;
    }

    @Override
    public void periodic() {
        // // 1. Calculate how much motor power we need to reach the target MM
        // double currentMm = getFilteredHoodDistance();
        // double pidOutput = m_hoodPID.calculate(currentMm, m_targetHoodDistanceMm);
        
        // // 2. Apply that power to the motor (DutyCycleOut is -1.0 to 1.0)
        // m_hoodMotor.setControl(new DutyCycleOut(pidOutput));

        SmartDashboard.putNumber("Shooter/Flywheel Vel", getFlywheelVelocity());
        SmartDashboard.putNumber("Shooter/Flywheel Target", m_targetFlywheelVelocity);
        
        SmartDashboard.putNumber("Shooter/Hood Position", getHoodPosition());
        SmartDashboard.putNumber("Shooter/Hood Target", m_targetHoodPosition);

        SmartDashboard.putNumber("Feeder Vel", getFeederVelocity());
        
        SmartDashboard.putNumber("Shooter/Hood Dist (mm)", getFilteredHoodDistance());
    }
}