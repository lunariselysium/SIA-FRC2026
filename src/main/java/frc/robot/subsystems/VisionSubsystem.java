package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("Vision");

    // --- FILTERS ---
    // MedianFilter rejects "outliers" (random spikes). Size 5 = looks at last 5 frames (0.1s)
    private final MedianFilter m_distMedianFilter = new MedianFilter(5);
    private final MedianFilter m_txMedianFilter = new MedianFilter(5);

    // LinearFilter smooths out the remaining noise (moving average).
    // Time constant 0.1s means it averages roughly the last 0.1s of data.
    private final LinearFilter m_distSmoother = LinearFilter.singlePoleIIR(0.1, 0.02);
    private final LinearFilter m_txSmoother = LinearFilter.singlePoleIIR(0.1, 0.02);

    private double m_filteredTx = 0.0;
    private double m_filteredDist = 0.0;

    public double getTx() {
        return m_filteredTx+5; // Return the smoothed value
    }

    public double getDistance() {
        return m_filteredDist; // Return the smoothed value
    }

    public double getRawTx() {
        return visionTable.getEntry("tx").getDouble(0.0);
    }
    
    public boolean hasTarget() {
        return visionTable.getEntry("has_target").getBoolean(false);
    }

    @Override
    public void periodic() {
        double rawTx = visionTable.getEntry("tx").getDouble(0.0);
        double rawDist = visionTable.getEntry("distance").getDouble(0.0);
        boolean hasTarget = hasTarget();

        // Only filter if we actually have a target, otherwise reset/hold
        if (hasTarget) {
            // 1. Remove spikes (Median)
            double medianTx = m_txMedianFilter.calculate(rawTx);
            double medianDist = m_distMedianFilter.calculate(rawDist);

            // 2. Smooth noise (IIR / Moving Average)
            m_filteredTx = m_txSmoother.calculate(medianTx);
            m_filteredDist = m_distSmoother.calculate(medianDist);
        } else {
            // Optional: You could reset filters here, or hold last value
            // For now, we leave them holding the last valid value to prevent "snap back" to 0
        }

        SmartDashboard.putBoolean("Vision/HasTarget", hasTarget);
        SmartDashboard.putNumber("Vision/Raw Tx", rawTx);
        SmartDashboard.putNumber("Vision/Filtered Tx", m_filteredTx);
        SmartDashboard.putNumber("Vision/Filtered Dist", m_filteredDist);
    }
}