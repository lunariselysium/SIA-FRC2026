package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterAngleDistanceSubsystem extends SubsystemBase{
    private final CANrange front_range = new CANrange(0);
    private static final double kMmPerMeter = 1000.0;
    private static final double kAlpha = 0.05;
    private double m_filteredDistance = 0;
    private boolean m_firstReading = true;

    public ShooterAngleDistanceSubsystem(){
    }
    
    public double getDistance() {
        return front_range.getDistance().getValueAsDouble() * kMmPerMeter;
    }

    private double getFilteredDistance() {
        double rawDistance = front_range.getDistance().getValueAsDouble() * kMmPerMeter;
        
        if (m_firstReading) {
            m_filteredDistance = rawDistance;
            m_firstReading = false;
        } else {
            m_filteredDistance = kAlpha * rawDistance + (1 - kAlpha) * m_filteredDistance;
        }
        
        return m_filteredDistance;
    }

    @Override
    public void periodic() {
	    SmartDashboard.putNumber("ShooterAngle Distance (mm)", getFilteredDistance());
	}
}
