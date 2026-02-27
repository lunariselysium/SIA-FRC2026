package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollerSubsystem extends SubsystemBase {
    private final TalonFX m_rollerMotor;

    private final VelocityVoltage m_velocityControl = new VelocityVoltage(0).withSlot(0);

    private static final int kRollerMotorId = 13;

    private double m_targetVelocity = 0;

    public IntakeRollerSubsystem() {
        m_rollerMotor = new TalonFX(kRollerMotorId);
        configureMotor();
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_rollerMotor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.print("Could not apply configs, error code:" + status.toString());
        }

        m_rollerMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setVelocity(double velocity) {
        m_targetVelocity = velocity;
        m_rollerMotor.setControl(m_velocityControl.withVelocity(velocity));
    }

    public void stop() {
        m_targetVelocity = 0;
        m_rollerMotor.setControl(new DutyCycleOut(0));
    }

    public double getVelocity() {
        return m_rollerMotor.getVelocity().getValueAsDouble();
    }

    public double getTargetVelocity() {
        return m_targetVelocity;
    }

    @Override
    public void periodic() {
    }
}
