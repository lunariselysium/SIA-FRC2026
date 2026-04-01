// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem.HoodPosition;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

import frc.robot.commands.ShooterCommands;
import frc.robot.commands.AlignToWallAndReset;
import frc.robot.commands.AutoAimAndShootCommand;
import frc.robot.commands.IntakeOscillateCommand;
import frc.robot.commands.ClimberCommands;

public class RobotContainer {
    private double MaxSpeed = 0.75 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private boolean inverted = false;

    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem(); 
    private final ClimberSubsystem climber = new ClimberSubsystem();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("AlignRightTrench", 
            new AlignToWallAndReset(drivetrain, 
                AlignToWallAndReset.RIGHT_SENSOR, 
                0.26, 
                null, 
                0.33+0.26
            ).withTimeout(5.0)
        );
        NamedCommands.registerCommand("AlignRightTrenchWithBack", 
            new AlignToWallAndReset(drivetrain, 
                AlignToWallAndReset.BACK_SENSOR, 
                0.26, 
                null, 
                0.33+0.26
            ).withTimeout(5.0)
        );
        NamedCommands.registerCommand("AlignLeftTrench", 
            new AlignToWallAndReset(drivetrain, 
                AlignToWallAndReset.RIGHT_SENSOR, 
                0.26, 
                null, 
                8.07-(0.33+0.26)
            ).withTimeout(5.0)
        );
        NamedCommands.registerCommand("TogglePivot", 
            Commands.runOnce(intake::togglePivot, intake)
        );
        NamedCommands.registerCommand("RunRollers", 
            Commands.runOnce(intake::runRollers, intake)
        );
        NamedCommands.registerCommand("StopRollers", 
            Commands.runOnce(intake::stopRollers, intake)
        );
        NamedCommands.registerCommand("AutoAimAndShootShort", 
            new AutoAimAndShootCommand(drivetrain, shooter, vision)
            .alongWith(
                new IntakeOscillateCommand(intake)
            )
            .withTimeout(3.0)
        );
        NamedCommands.registerCommand("AutoAimAndShoot", 
            new AutoAimAndShootCommand(drivetrain, shooter, vision)
            .alongWith(
                new SequentialCommandGroup(
                    new WaitCommand(1.5),
                    new IntakeOscillateCommand(intake)
                )
            )
            .withTimeout(10.0)
        );
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                // Drivetrain will execute this command periodically
                double multiplier = inverted ? -1.0 : 1.0;
                return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * multiplier)
                            .withVelocityY(-joystick.getLeftX() * MaxSpeed * multiplier)
                            .withRotationalRate(-joystick.getRightX() * MaxAngularRate);
            })
        );
        
        shooter.setDefaultCommand(
            Commands.run(
                () -> {
                    shooter.setFlywheelVelocity(50.0); // Idle speed
                    shooter.stopFeeder();
                }, 
                shooter
            )
        );

        climber.setDefaultCommand(
            ClimberCommands.manualMove(climber, () -> -joystick.getRightY())
        );

        // joystick.leftBumper().whileTrue(
        //     new AutoAimAndShootCommand(drivetrain, shooter, vision)
        //     .alongWith(
        //         new SequentialCommandGroup(
        //             new WaitCommand(1.5),
        //             new IntakeOscillateCommand(intake)
        //         )
        //     )
        // );
        // joystick.leftTrigger(0.5).whileTrue(getPassCommand());
        // joystick.rightBumper().whileTrue(
        //     Commands.startEnd(
        //         intake::runRollers,   // Runs when button is pressed
        //         intake::stopRollers,  // Runs automatically when button is released
        //         intake                // Requires the intake subsystem
        //     )
        // );
        // joystick.rightTrigger(0.5).whileTrue(
        //     Commands.startEnd(intake::runRollersReverse, intake::stopRollers, intake)
        //     .alongWith(
        //         ShooterCommands.runFeederReverse(shooter)
        //     )
        // );
        joystick.x().onTrue(Commands.runOnce(intake::togglePivot, intake));
        joystick.y().onTrue(Commands.runOnce(() -> inverted = !inverted));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );      
        //    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //    // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //    //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        //    // ));
        //    joystick.a().onTrue(ClimberCommands.retractToBottom(climber));
        //    joystick.b().onTrue(ClimberCommands.extendToTop(climber));   
        //    // Run SysId routines when holding back/start and X/Y.
        //    // Note that each routine should be run exactly once in a single log.
        //    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));   
        //    // Reset the field-centric heading on y press.
        //    // joystick.y().onTrue((drivetrain.runOnce(drivetrain::seedFieldCentric)));      
        //    // --- INTAKE BINDINGS ---
                  
        //    // X Button: Toggle Intake
        //    joystick.x().onTrue(Commands.runOnce(intake::togglePivot, intake));      
        //    // Left Bumper: Run Roller
        //    joystick.leftBumper().toggleOnTrue(Commands.startEnd(intake::runRollers, intake::stopRollers, intake));      
        //    // Left Trigger: Oscillate/Shake Intake (Pivot Bobbing)
        //    // Runs as long as trigger is held past 50%
        //    joystick.leftTrigger(0.5).whileTrue(new IntakeOscillateCommand(intake));     
           // 1. Right Bumper (Button on top towards user) -> Run Feeder
           joystick.rightBumper()
               .whileTrue(ShooterCommands.runFeeder(shooter));      
        //    // 2. POV Right (D-Pad Right) -> Increase Flywheel Speed
        //    // We use onTrue so you have to click it to step up (prevents zooming to max speed instantly)
        //    joystick.povRight()
        //        .onTrue(ShooterCommands.increaseFlywheelSpeed(shooter));     
        //    // 3. POV Left (D-Pad Left) -> Decrease Flywheel Speed
        //    joystick.povLeft()
        //        .onTrue(ShooterCommands.decreaseFlywheelSpeed(shooter));     
        //    // 4. POV Up (D-Pad Up) -> Move Hood Up
        //    // We use whileTrue so it moves smoothly while holding the button
        //    joystick.povUp()
        //        .whileTrue(ShooterCommands.moveHoodUp(shooter));     
        //    // 5. POV Down (D-Pad Down) -> Move Hood Down
        //    joystick.povDown()
        //        .whileTrue(ShooterCommands.moveHoodDown(shooter));

        //     drivetrain.registerTelemetry(logger::telemeterize);

        // joystick.rightTrigger(0.5).whileTrue(
        //     new AutoAimAndShootCommand(drivetrain, shooter, vision)
        // );
    }

    public Command getPassCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                shooter.setFlywheelVelocity(70.0*0.8);
                // shooter.setFlywheelVelocity(20.0);

                shooter.setHoodState(HoodPosition.HIGH);
            }, shooter),
            Commands.waitUntil(() -> shooter.isHoodAtPosition(1) && shooter.isFlywheelAtSpeed(5)),
            Commands.parallel(
                // ShooterCommands.runFeeder(shooter),
                shooter.run(() -> shooter.runFeeder(100)), 
                new IntakeOscillateCommand(intake)
            )
        ).finallyDo((interrupted) -> {
            shooter.stopFeeder(); 
            shooter.setFlywheelVelocity(50.0);
            // shooter.setFlywheelVelocity(20.0);

            shooter.setHoodState(HoodPosition.LOW);
        });
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
