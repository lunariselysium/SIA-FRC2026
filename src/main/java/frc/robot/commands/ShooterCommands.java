package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommands {

    // --- TUNING CONSTANTS ---
    // How much to change speed per button press (e.g., 500 RPM)
    private static final double FLYWHEEL_STEP =5.0; 
    // How much to move hood per button press
    private static final double HOOD_STEP = 0.5; 
    // Speed to run the feeder
    private static final double FEEDER_SPEED = 15.0; 

    private ShooterCommands() {
        // Private constructor because this is a factory class
    }

    /**
     * Runs the feeder wheels while the command is active.
     * Stops them when the command ends (button released).
     */
    public static Command runFeeder(ShooterSubsystem shooter) {
        // 1. Wait until the flywheel is at speed
        return Commands.waitUntil(() -> shooter.isFlywheelAtSpeed(3))
            // 2. Once at speed, run the feeder until the command ends (button released)
            .andThen(
                Commands.startEnd(
                    () -> shooter.runFeeder(FEEDER_SPEED),
                    () -> shooter.stopFeeder(),
                    shooter
                )
            );
    }

    /**
     * Increases the Flywheel Target Velocity by a fixed step.
     */
    public static Command increaseFlywheelSpeed(ShooterSubsystem shooter) {
        return Commands.runOnce(() -> {
            double current = shooter.getTargetFlywheelVelocity(); // Or getTargetFlywheelVelocity()
            shooter.setFlywheelVelocity(current + FLYWHEEL_STEP);
        }, shooter);
    }

    /**
     * Decreases the Flywheel Target Velocity by a fixed step.
     */
    public static Command decreaseFlywheelSpeed(ShooterSubsystem shooter) {
        return Commands.runOnce(() -> {
            double current = shooter.getTargetFlywheelVelocity(); // Or getTargetFlywheelVelocity()
            // Don't let it go below 0
            double newSpeed = Math.max(0, current - FLYWHEEL_STEP);
            shooter.setFlywheelVelocity(newSpeed);
        }, shooter);
    }

    /**
     * Moves the hood UP (Increase position).
     * We use run() so it keeps moving as long as you hold the button
     * (Repeatedly adding the step).
     */
    public static Command moveHoodUp(ShooterSubsystem shooter) {
        return Commands.run(() -> {
            shooter.setHoodDistanceMm(shooter.getHoodTargetDistance()+HOOD_STEP);
        }, shooter);
    }

    /**
     * Moves the hood DOWN (Decrease position).
     */
    public static Command moveHoodDown(ShooterSubsystem shooter) {
        return Commands.run(() -> {
            shooter.setHoodDistanceMm(shooter.getHoodTargetDistance()-HOOD_STEP);
        }, shooter);
    }
    
    /**
     * Emergency Stop for the shooter
     */
    public static Command stopShooter(ShooterSubsystem shooter) {
        return Commands.runOnce(() -> shooter.stopEverything(), shooter);
    }
}