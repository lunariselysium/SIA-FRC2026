package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.MathUtil;;

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
    private final VelocityVoltage m_flywheelControl
         = new VelocityVoltage(0).withEnableFOC(false).withSlot(0);
    private final VelocityVoltage m_feederControl = new VelocityVoltage(0).withSlot(0);

    private final edu.wpi.first.math.controller.PIDController m_hoodPID = 
        new edu.wpi.first.math.controller.PIDController(0.00024, 0.000001, 0.00010);
    private static final double kHoodPIDkS = 0.03;
    private static final double kMaxHoodOutput = 0.20;

    // --- STATE VARIABLES ---
    private double m_targetFlywheelVelocity = 0;
    private double m_targetHoodDistanceMm = 81; // CANrange data
    
    // Distance Sensor Filter logic
    private static final double kMmPerMeter = 1000.0;

    // 1D Kalman Filter Variables
    private static final double kQ = 0.1;  // Process Noise: How fast the actual distance can physically change (Tune this)
    private static final double kR = 500.0; // Measurement Noise: How "noisy" you expect the CANrange to be (Tune this)
    private double m_kalmanP = 1.0;        // Error covariance estimate
    private double m_kalmanX = 0.0;        // The filtered distance estimate
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
        config.Slot0.kS = 0.5;
        config.Slot0.kV = 0.108;
        config.Slot0.kP = 0.4;
        config.Slot0.kI = 0.05;
        config.Slot0.kD = 0.0;

        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;

        // Stator limit is current in the motor itself (limits acceleration/torque)
        config.CurrentLimits.StatorCurrentLimit = 100; // Max 60 Amps
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // Supply limit is current drawn from the battery (keeps main breaker happy)
        config.CurrentLimits.SupplyCurrentLimit = 50; 
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        applyConfig(m_leftFlywheel, config, "Left Flywheel");

        // Set Right to follow Left, but spinning the opposite direction
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
        config.Voltage.PeakForwardVoltage = 8;
        config.Voltage.PeakReverseVoltage = -8;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        applyConfig(m_hoodMotor, config, "Hood Motor");
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
     * Sets the target position for the Hood.
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

    public double getTargetFlywheelVelocity() {
        return m_targetFlywheelVelocity;
    }

    public double getFeederVelocity() {
        return m_feederMotor.getVelocity().getValueAsDouble();
    }

    public double getHoodTargetDistance() {
        return m_targetHoodDistanceMm;
    }

    public double getFilteredHoodDistance() {
        double rawDistance = Math.round(m_hoodRange.getDistance().getValueAsDouble() * kMmPerMeter);
        
        // --- OUTLIER REJECTION ---
        // If the sensor reads 0 or an impossibly far distance, ignore this loop entirely.
        if (rawDistance <= 50.0 || rawDistance > 200.0) {
            return m_kalmanX; // Just return the last known good estimate
        }

        if (m_firstReading) {
            // Initialize the filter with the first reading
            m_kalmanX = rawDistance;
            m_firstReading = false;
        } else {
            // --- 1. Prediction Step ---
            m_kalmanP = m_kalmanP + kQ;

            // --- 2. Update Step ---
            double kalmanGain = m_kalmanP / (m_kalmanP + kR);
            m_kalmanX = m_kalmanX + kalmanGain * (rawDistance - m_kalmanX);
            m_kalmanP = (1.0 - kalmanGain) * m_kalmanP;
        }

        return m_kalmanX;
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
    public boolean isHoodAtDistance(double toleranceMm) {
        return Math.abs(getFilteredHoodDistance() - m_targetHoodDistanceMm) <= toleranceMm;
    }


    @Override
    public void periodic() {
        // 1. Get current distance
        double currentMm = getFilteredHoodDistance();

        // 2. Calculate PID output (Voltage or % output)
        double pidOutput = m_hoodPID.calculate(currentMm, m_targetHoodDistanceMm);

        // 3. Add Feedforward (kS) to overcome friction
        // If PID wants to go positive, add kS. If negative, subtract kS.
        if (pidOutput > 0.001) {
            pidOutput += kHoodPIDkS;
        } else if (pidOutput < -0.001) {
            pidOutput -= kHoodPIDkS;
        }

        // 4. Clamp output
        double clampedOutput = MathUtil.clamp(pidOutput, -kMaxHoodOutput, kMaxHoodOutput);
        
        // 5. Apply to motor
        m_hoodMotor.setControl(new DutyCycleOut(clampedOutput));

        SmartDashboard.putNumber("Shooter/Flywheel Vel", getFlywheelVelocity());
        SmartDashboard.putNumber("Shooter/Flywheel Target", m_targetFlywheelVelocity);
        
        SmartDashboard.putNumber("Shooter/Hood Position", getFilteredHoodDistance());
        SmartDashboard.putNumber("Shooter/Hood Target", m_targetHoodDistanceMm);

        SmartDashboard.putNumber("Feeder Vel", getFeederVelocity());
        
        SmartDashboard.putNumber("Shooter/Hood Dist (mm)", getFilteredHoodDistance());
    }
}