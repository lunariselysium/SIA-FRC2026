package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAimAndShootCommand extends Command {

    private final CommandSwerveDrivetrain m_drivetrain;
    private final ShooterSubsystem m_shooter;
    private final VisionSubsystem m_vision;

    // --- CTRE SWERVE REQUEST ---
    // We use RobotCentric because we want to lock X/Y movement completely relative to the robot frame.
    private final SwerveRequest.RobotCentric m_driveReq = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Open Loop is snappier for aiming
            .withDeadband(0.05)
            .withRotationalDeadband(0.05);

    // --- PID CONTROLLER ---
    private final PIDController m_turnPID = new PIDController(0.01, 0.0, 0.001);
    private final double kTurnToleranceDeg = 3.0;

    // --- LOOKUP TABLES ---
    private final InterpolatingDoubleTreeMap m_rpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_hoodMap = new InterpolatingDoubleTreeMap();

    // --- FEEDER CONSTANT ---
    private static final double FEEDER_SPEED_RPS = 25.0;

    public AutoAimAndShootCommand(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter, VisionSubsystem vision) {
        m_drivetrain = drivetrain;
        m_shooter = shooter;
        m_vision = vision;

        // REQUIRE the subsystems so no other command can mess with them while we aim
        addRequirements(drivetrain, shooter); 
        // Note: We don't strictly need to require Vision, as we only read from it.

        configureLookupTables();
        m_turnPID.setTolerance(kTurnToleranceDeg);
    }

    private void configureLookupTables() {
        // --- DATA FROM YOU ---
        // 2.0m -> 55 RPS, 82 Hood
        // 2.5m -> 50 RPS, 94 Hood

        m_rpmMap.put(1.5, 49.0);
        m_rpmMap.put(2.0, 56.0);
        m_rpmMap.put(2.5, 58.0);
        
        // --- EXTRAPOLATION (Guessing to fill gaps) ---
        // Close range (Manual shot equivalent?)
        // m_rpmMap.put(1.0, 55.0); 
        
        // // Far range
        // m_rpmMap.put(3.0, 48.0); // Physics says spin usually goes UP with distance, but your data says DOWN. I'll trust your data trend.
        // m_rpmMap.put(4.0, 45.0);

        // m_hoodMap.put(1.0, 70.0);
        m_hoodMap.put(1.5,82.0);
        m_hoodMap.put(2.0, 84.0);
        m_hoodMap.put(2.5, 92.0);
        // m_hoodMap.put(3.0, 105.0);
        // m_hoodMap.put(4.0, 120.0);
    }

    @Override
    public void initialize() {
        m_turnPID.reset();
        SmartDashboard.putString("AutoAim/Status", "Initializing...");
    }

    @Override
    public void execute() {
        // // 1. SAFETY: If no target, stop everything
        // if (!m_vision.hasTarget()) {
            // m_drivetrain.setControl(m_driveReq.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            // m_shooter.stopFeeder();
        //     // Optional: Idle flywheel at base speed?
        //     m_shooter.setFlywheelVelocity(55); 
        //     // m_shooter.setFlywheelVelocity(10);
        //     SmartDashboard.putString("AutoAim/Status", "NO TARGET");
        //     return;
        // }
        m_drivetrain.setControl(m_driveReq.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        m_shooter.stopFeeder();

        // 2. GET DATA
        // Note: Using filtered values from your VisionSubsystem is CRITICAL here
        double dist = m_vision.getDistance();
        double tx = m_vision.getTx();

        // 3. CALCULATE SETPOINTS
        // Clamp distance to avoid looking up crazy values if vision glitches to 0.0 or 50.0m
        double safeDist = Math.max(1.0, Math.min(dist, 4.0));
        
        double targetRPM = m_rpmMap.get(safeDist);
        double targetHood = m_hoodMap.get(safeDist);

        m_shooter.setFlywheelVelocity(targetRPM);
        m_shooter.setHoodDistanceMm(targetHood);

        // 4. CALCULATE TURNING
        double rotationSpeed = m_turnPID.calculate(tx, 0);
        
        // Clamp rotation speed so we don't spin violently (Max 50% speed)
        rotationSpeed = Math.max(-0.5, Math.min(rotationSpeed, 0.5));

        // 5. APPLY DRIVETRAIN
        // This is the "Lock Translation" part. Velocity X and Y are 0.
        m_drivetrain.setControl(
            m_driveReq
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(rotationSpeed * Math.PI * 2) // Convert to Radians/Sec if needed, Phoenix uses Radians usually
        );

        // 6. CHECK READY STATE
        boolean isAimed = m_turnPID.atSetpoint();
        boolean isSpedUp = m_shooter.isFlywheelAtSpeed(3.0); // 3 RPS tolerance
        boolean isHooded = m_shooter.isHoodAtDistance(3.0); // 2mm tolerance

        SmartDashboard.putBoolean("AutoAim/Ready: Aim", isAimed);
        SmartDashboard.putBoolean("AutoAim/Ready: RPM", isSpedUp);

        // 7. FEEDER LOGIC
        // Only run feeder if we are aimed AND sped up AND hood is there.
        if (isAimed && isSpedUp && isHooded) {
            m_shooter.runFeeder(FEEDER_SPEED_RPS);
            SmartDashboard.putString("AutoAim/Status", "FIRING");
        } else {
            m_shooter.stopFeeder();
            SmartDashboard.putString("AutoAim/Status", "Aligning...");
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop Everything
        m_drivetrain.setControl(m_driveReq.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        m_shooter.stopFeeder();
        SmartDashboard.putString("AutoAim/Status", "Ended");
    }

    @Override
    public boolean isFinished() {
        return false; // Run until button released
    }
}