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

public class FalconSubsystem extends SubsystemBase {
    private final TalonFX m_leftFalcon;
    private final TalonFX m_rightFalcon;

    private final VelocityVoltage m_leftFalconControl = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage m_rightFalconControl = new VelocityVoltage(0).withSlot(0);

    private static final int kLeftFalconId = 9;
    private static final int kRightFalconId = 10;

    private double m_targetVelocity = 0;

    public FalconSubsystem() {
        m_leftFalcon = new TalonFX(kLeftFalconId);
        m_rightFalcon = new TalonFX(kRightFalconId);
        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration falconConfig = new TalonFXConfiguration();
        
        falconConfig.Slot0.kP = 0.1;
        falconConfig.Slot0.kI = 0;
        falconConfig.Slot0.kD = 0;
        falconConfig.Voltage.PeakForwardVoltage = 12;
        falconConfig.Voltage.PeakReverseVoltage = -12;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_leftFalcon.getConfigurator().apply(falconConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.print("Could not apply falcon configs, error code:" + status.toString());
        }

        m_rightFalcon.setControl(new Follower(kLeftFalconId, MotorAlignmentValue.Opposed));

        m_leftFalcon.setNeutralMode(NeutralModeValue.Coast);
        m_rightFalcon.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setVelocity(double velocity) {
        m_targetVelocity = velocity;
        m_leftFalcon.setControl(m_leftFalconControl.withVelocity(velocity));
    }

    public void stop() {
        m_targetVelocity = 0;
        m_leftFalcon.setControl(new DutyCycleOut(0));
        m_rightFalcon.setControl(new DutyCycleOut(0));
    }

    public double getLeftFalconVelocity() {
        return m_leftFalcon.getVelocity().getValueAsDouble();
    }

    public double getRightFalconVelocity() {
        return m_rightFalcon.getVelocity().getValueAsDouble();
    }

    public double getTargetVelocity() {
        return m_targetVelocity;
    }

    @Override
    public void periodic() {
    }
}
