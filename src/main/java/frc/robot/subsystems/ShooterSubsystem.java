package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX m_feederKraken;
    private final TalonFX m_shooterKraken;

    private final VelocityVoltage m_feederKrakenControl = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage m_shooterKrakenControl = new VelocityVoltage(0).withSlot(0);

    private static final int kFeederKrakenId = 16;
    private static final int kShooterKrakenId = 11;

    private double m_targetKrakenVelocity = 0;

    public ShooterSubsystem() {
        m_feederKraken = new TalonFX(kFeederKrakenId);
        m_shooterKraken = new TalonFX(kShooterKrakenId);
        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration krakenConfig = new TalonFXConfiguration();
        
        krakenConfig.Slot0.kP = 0.1;
        krakenConfig.Slot0.kI = 0;
        krakenConfig.Slot0.kD = 0;
        krakenConfig.Voltage.PeakForwardVoltage = 12;
        krakenConfig.Voltage.PeakReverseVoltage = -12;
        krakenConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_feederKraken.getConfigurator().apply(krakenConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.print("Could not apply kraken configs, error code:" + status.toString());
        }

        m_feederKraken.setNeutralMode(NeutralModeValue.Coast);
        m_shooterKraken.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setKrakenVelocity(double velocity) {
        m_targetKrakenVelocity = velocity;
        m_feederKraken.setControl(m_feederKrakenControl.withVelocity(velocity));
        m_shooterKraken.setControl(m_shooterKrakenControl.withVelocity(velocity));
    }

    public void setShooter(double krakenVelocity) {
        setKrakenVelocity(krakenVelocity);
    }

    public void stop() {
        m_targetKrakenVelocity = 0;
        m_feederKraken.setControl(new DutyCycleOut(0));
        m_shooterKraken.setControl(new DutyCycleOut(0));
    }

    public double getFeederKrakenVelocity() {
        return m_feederKraken.getVelocity().getValueAsDouble();
    }

    public double getShooterKrakenVelocity() {
        return m_shooterKraken.getVelocity().getValueAsDouble();
    }

    public double getTargetKrakenVelocity() {
        return m_targetKrakenVelocity;
    }

    @Override
    public void periodic() {
    }
}
