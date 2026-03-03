package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("Vision");

    public double getTx() {
        return visionTable.getEntry("tx").getDouble(0.0);
    }

    public double getDistance() {
        return visionTable.getEntry("distance").getDouble(0.0);
    }

    public boolean hasTarget() {
        return visionTable.getEntry("has_target").getBoolean(false);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Vision Has Target", hasTarget());
        SmartDashboard.putNumber("Vision Tx", getTx());
        SmartDashboard.putNumber("Vision Distance", getDistance());
    }
}