package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    // --- Hardware ---
    private final TalonFX m_pivotMotor;
    private final TalonFX m_rollerMotor;

    // --- Control Requests ---
    private final PositionVoltage m_pivotControl = new PositionVoltage(0).withSlot(0);
    private final VelocityVoltage m_rollerControl = new VelocityVoltage(0).withSlot(0);
    private final DutyCycleOut m_stopControl = new DutyCycleOut(0);

    // --- Constants & IDs ---
    private static final int kPivotMotorId = 14;
    private static final int kRollerMotorId = 13;

    // --- Default Settings (Hardcoded but Modifiable) ---
    // Pivot Positions (Rotations)
    public double m_posUp = 0.0;
    public double m_posDown = 13.0;
    public double m_posOscillate = 6.0; // The "Partial" retract position

    // Roller Speeds (Rotations Per Second)
    public double m_rollerSpeed = 100.0; 

    // State
    private double m_targetPivotPosition = 0;
    private double m_targetRollerVelocity = 0;

    public IntakeSubsystem() {
        m_pivotMotor = new TalonFX(kPivotMotorId);
        m_rollerMotor = new TalonFX(kRollerMotorId);

        configurePivotMotor();
        configureRollerMotor();

        // Initialize Dashboard with defaults
        SmartDashboard.putNumber("Intake/Pivot Up Pos", m_posUp);
        SmartDashboard.putNumber("Intake/Pivot Down Pos", m_posDown);
        SmartDashboard.putNumber("Intake/Pivot Oscillate Pos", m_posOscillate);
        SmartDashboard.putNumber("Intake/Roller Speed RPS", m_rollerSpeed);
    }

    private void configurePivotMotor() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        // Pivot PID
        configs.Slot0.kP = 0.3;
        configs.Slot0.kI = 0.05;
        configs.Slot0.kD = 0.1;
        
        configs.Voltage.PeakForwardVoltage = 8;
        configs.Voltage.PeakReverseVoltage = -8;
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 

        m_pivotMotor.getConfigurator().apply(configs);
        m_pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        m_pivotMotor.setPosition(0);
    }

    private void configureRollerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Roller PID
        config.Slot0.kP = 0.4;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        
        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;

        m_rollerMotor.getConfigurator().apply(config);
        m_rollerMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    // --- Pivot Methods ---
    public void setPivotPosition(double position) {
        m_targetPivotPosition = position;
        m_pivotMotor.setControl(m_pivotControl.withPosition(position));
    }

    public void pivotUp() {
        setPivotPosition(m_posUp);
    }

    public void pivotDown() {
        setPivotPosition(m_posDown);
    }

    public void pivotOscillate() {
        setPivotPosition(m_posOscillate);
    }

    public double getPivotPosition() {
        return m_pivotMotor.getPosition().getValueAsDouble();
    }

    public void togglePivot() {
        // If we are currently trying to go Down (or Oscillate), go Up.
        // If we are Up, go Down.
        if (Math.abs(m_targetPivotPosition - m_posUp) < 0.1) {
            pivotDown();
        } else {
            pivotUp();
        }
    }

    // --- Roller Methods ---
    public void setRollerVelocity(double rps) {
        m_targetRollerVelocity = rps;
        m_rollerMotor.setControl(m_rollerControl.withVelocity(rps));
    }

    public void runRollers() {
        setRollerVelocity(-m_rollerSpeed);
    }

    public void stopRollers() {
        m_targetRollerVelocity = 0;
        m_rollerMotor.setControl(m_stopControl);
    }

    public double getRollerVelocity() {
        return m_rollerMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // Update variables from Dashboard (allows Tuning)
        m_posUp = SmartDashboard.getNumber("Intake/Pivot Up Pos", m_posUp);
        m_posDown = SmartDashboard.getNumber("Intake/Pivot Down Pos", m_posDown);
        m_posOscillate = SmartDashboard.getNumber("Intake/Pivot Oscillate Pos", m_posOscillate);
        m_rollerSpeed = SmartDashboard.getNumber("Intake/Roller Speed RPS", m_rollerSpeed);

        // Telemetry
        SmartDashboard.putNumber("Intake/Real Pivot Pos", getPivotPosition());
        SmartDashboard.putNumber("Intake/Real Roller Vel", getRollerVelocity());
    }
}