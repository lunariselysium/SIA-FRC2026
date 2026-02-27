package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterAngleSubsystem extends SubsystemBase {
    private final TalonFX m_angleMotor;

    private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

    private static final int kAngleMotorId = 15;

    private double m_targetPosition = 0;
    private static final double kStepSize = 1.0;

    public ShooterAngleSubsystem() {
        m_angleMotor = new TalonFX(kAngleMotorId);
        configureMotor();
    }

    private void configureMotor() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        configs.Slot0.kP = 1;
        configs.Slot0.kI = 0;
        configs.Slot0.kD = 0.1;
        configs.Voltage.PeakForwardVoltage = 8;
        configs.Voltage.PeakReverseVoltage = -8;
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_angleMotor.getConfigurator().apply(configs);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.print("Could not apply configs, error code:" + status.toString());
        }

        m_angleMotor.setNeutralMode(NeutralModeValue.Brake);
        m_angleMotor.setPosition(0);
    }

    public void setPosition(double position) {
        m_targetPosition = position;
        m_angleMotor.setControl(m_positionVoltage.withPosition(position));
    }

    public void increase() {
        setPosition(m_targetPosition + kStepSize);
    }

    public void decrease() {
        setPosition(m_targetPosition - kStepSize);
    }

    public void stop() {
        m_angleMotor.setControl(new com.ctre.phoenix6.controls.DutyCycleOut(0));
    }

    public double getPosition() {
        return m_angleMotor.getPosition().getValueAsDouble();
    }

    public double getTargetPosition() {
        return m_targetPosition;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Angle", getPosition());
        SmartDashboard.putNumber("Shooter Angle Target", m_targetPosition);
    }
}
