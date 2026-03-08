package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommands {

    private ClimberCommands() {}

    /**
     * Manual Move Command. 
     * Useful for binding to a joystick axis or D-Pad.
     */
    public static Command manualMove(ClimberSubsystem climber, DoubleSupplier speedSupplier) {
        return Commands.run(
            () -> climber.setPower(speedSupplier.getAsDouble()), 
            climber
        );
        // ).finallyDo(() -> climber.stop());
    }

    /**
     * Extends the climber to the top position (Ready to grab chain).
     */
    public static Command extendToTop(ClimberSubsystem climber) {
        return Commands.run(
            () -> climber.setPosition(ClimberSubsystem.kPositionUp), 
            climber
        ).until(() -> climber.atSetpoint(ClimberSubsystem.kPositionUp, 2.0)); 
        // Note: We usually keep the command running to "Hold" the position via PID,
        // but if you want it to finish so you can do other things, remove the .until() logic
        // and just let it run.
    }

    /**
     * Retracts the climber to 0 (Pulls robot up).
     */
    public static Command retractToBottom(ClimberSubsystem climber) {
        return Commands.run(
            () -> climber.setPosition(ClimberSubsystem.kPositionDown), 
            climber
        );
    }
    
    /**
     * Zeros the encoder. Use this if the climber slips or starts in wrong spot.
     */
    public static Command zeroClimber(ClimberSubsystem climber) {
        return Commands.runOnce(() -> climber.resetEncoder(), climber);
    }
}