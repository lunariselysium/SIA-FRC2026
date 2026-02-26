package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX m_intakeMotor;

    private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

    private static final int kIntakeMotorId = 14;

    private double m_targetPosition = 0;
    private boolean m_isUp = false;

    private static final double kUpPosition = 0;
    private static final double kDownPosition = 1.0;

    public IntakeSubsystem() {
        m_intakeMotor = new TalonFX(kIntakeMotorId);
        configureMotor();
    }

    private void configureMotor() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        configs.Slot0.kP = 1;
        configs.Slot0.kI = 0;
        configs.Slot0.kD = 0.1;
        configs.Voltage.PeakForwardVoltage = 8;
        configs.Voltage.PeakReverseVoltage = -8;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_intakeMotor.getConfigurator().apply(configs);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.print("Could not apply configs, error code:" + status.toString());
        }

        m_intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        m_intakeMotor.setPosition(0);
    }

    public void setPosition(double position) {
        m_targetPosition = position;
        m_intakeMotor.setControl(m_positionVoltage.withPosition(position));
    }

    public void toggle() {
        if (m_isUp) {
            setPosition(kDownPosition);
            m_isUp = false;
        } else {
            setPosition(kUpPosition);
            m_isUp = true;
        }
    }

    public void raise() {
        setPosition(kUpPosition);
        m_isUp = true;
    }

    public void lower() {
        setPosition(kDownPosition);
        m_isUp = false;
    }

    public void stop() {
        m_intakeMotor.setControl(new com.ctre.phoenix6.controls.DutyCycleOut(0));
    }

    public double getPosition() {
        return m_intakeMotor.getPosition().getValueAsDouble();
    }

    public double getTargetPosition() {
        return m_targetPosition;
    }

    public boolean isUp() {
        return m_isUp;
    }

    @Override
    public void periodic() {
    }
}
