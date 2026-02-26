package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX m_leftFalcon;
    private final TalonFX m_rightFalcon;
    private final TalonFX m_feederKraken;
    private final TalonFX m_shootfeederKraken;

    private final VelocityVoltage m_leftFalconControl = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage m_rightFalconControl = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage m_feederKrakenControl = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage m_shootfeederKrakenControl = new VelocityVoltage(0).withSlot(0);

    private static final int kLeftFalconId = 10;
    private static final int kRightFalconId = 11;
    private static final int kFeederKrakenId = 12;
    private static final int kShooterKrakenId = 13;

    private double m_targetFalconVelocity = 0;
    private double m_targetKrakenVelocity = 0;

    public ShooterSubsystem() {
        m_leftFalcon = new TalonFX(kLeftFalconId);
        m_rightFalcon = new TalonFX(kRightFalconId);
        m_feederKraken = new TalonFX(kFeederKrakenId);
        m_shootfeederKraken = new TalonFX(kShooterKrakenId);

        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration falconConfig = new TalonFXConfiguration();
        
        falconConfig.Slot0.kP = 0.1;
        falconConfig.Slot0.kI = 0;
        falconConfig.Slot0.kD = 0;
        falconConfig.Voltage.PeakForwardVoltage = 12;
        falconConfig.Voltage.PeakReverseVoltage = -12;

        TalonFXConfiguration krakenConfig = new TalonFXConfiguration();
        
        krakenConfig.Slot0.kP = 0.1;
        krakenConfig.Slot0.kI = 0;
        krakenConfig.Slot0.kD = 0;
        krakenConfig.Voltage.PeakForwardVoltage = 12;
        krakenConfig.Voltage.PeakReverseVoltage = -12;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_leftFalcon.getConfigurator().apply(falconConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.print("Could not apply falcon configs, error code:" + status.toString());
        }

        status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_feederKraken.getConfigurator().apply(krakenConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.print("Could not apply kraken configs, error code:" + status.toString());
        }

        m_rightFalcon.setControl(new Follower(kLeftFalconId, MotorAlignmentValue.Opposed));

        m_leftFalcon.setNeutralMode(NeutralModeValue.Coast);
        m_rightFalcon.setNeutralMode(NeutralModeValue.Coast);
        m_feederKraken.setNeutralMode(NeutralModeValue.Coast);
        m_shootfeederKraken.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setFalconVelocity(double velocity) {
        m_targetFalconVelocity = velocity;
        m_leftFalcon.setControl(m_leftFalconControl.withVelocity(velocity));
    }

    public void setKrakenVelocity(double velocity) {
        m_targetKrakenVelocity = velocity;
        m_feederKraken.setControl(m_feederKrakenControl.withVelocity(velocity));
        m_shootfeederKraken.setControl(m_shootfeederKrakenControl.withVelocity(velocity));
    }

    public void setShooter(double falconVelocity, double krakenVelocity) {
        setFalconVelocity(falconVelocity);
        setKrakenVelocity(krakenVelocity);
    }

    public void stop() {
        m_targetFalconVelocity = 0;
        m_targetKrakenVelocity = 0;
        m_leftFalcon.setControl(new DutyCycleOut(0));
        m_rightFalcon.setControl(new DutyCycleOut(0));
        m_feederKraken.setControl(new DutyCycleOut(0));
        m_shootfeederKraken.setControl(new DutyCycleOut(0));
    }

    public double getLeftFalconVelocity() {
        return m_leftFalcon.getVelocity().getValueAsDouble();
    }

    public double getRightFalconVelocity() {
        return m_rightFalcon.getVelocity().getValueAsDouble();
    }

    public double getFeederKrakenVelocity() {
        return m_feederKraken.getVelocity().getValueAsDouble();
    }

    public double getShooterKrakenVelocity() {
        return m_shootfeederKraken.getVelocity().getValueAsDouble();
    }

    public double getTargetFalconVelocity() {
        return m_targetFalconVelocity;
    }

    public double getTargetKrakenVelocity() {
        return m_targetKrakenVelocity;
    }

    @Override
    public void periodic() {
    }
}
