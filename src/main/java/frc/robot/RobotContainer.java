// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AlignPosition;
import frc.robot.commands.AlignToAprilTag;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.RunIndexer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * RobotContainer for FRC 2026 REBUILT season This class is where the robot's subsystems, commands,
 * and button bindings are defined.
 */
public class RobotContainer {

  // Controllers
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);

  // State Machine
  private final RobotStateMachine m_stateMachine = RobotStateMachine.getInstance();

  // ==================== SUBSYSTEMS ====================
  // Drivetrain - created from TunerConstants
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // Vision
  public final LimelightSubsystem limelight = new LimelightSubsystem();
  // Indexer
  private final IndexerSubsystem indexer = new IndexerSubsystem();
  // Shooter
  private final ShooterSubsystem shooter = new ShooterSubsystem();

  // Telemetry - publishes robot pose to Elastic dashboard
  private final Telemetry logger = new Telemetry(Constants.DrivetrainConstants.MAX_SPEED_MPS);

  public RobotContainer() {
    // Link limelight to drivetrain for vision-based odometry
    limelight.setDrivetrain(drivetrain);

    // Register controllers with state machine for haptic feedback
    m_stateMachine.registerControllers(m_driverController, m_operatorController);

    // Configure button bindings
    configureBindings();
  }

  /**
   * Configure button bindings for driver and operator controllers This is where you bind controller
   * buttons to commands
   */
  private void configureBindings() {
    // Register telemetry to publish robot pose to NetworkTables for Elastic
    // dashboard
    drivetrain.registerTelemetry(logger::telemeterize);

    // ==================== DRIVER CONTROLS ====================
    drivetrain.setDefaultCommand(drivetrain.smoothTeleopDriveCommand(
        () -> m_driverController.getLeftY(), // Forward/backward
        () -> m_driverController.getLeftX(), // Left/right strafe
        () -> m_driverController.getRightX(), // Rotation
        Constants.DrivetrainConstants.MAX_SPEED_MPS,
        Constants.DrivetrainConstants.MAX_ANGULAR_RATE_RAD_PER_SEC));

    // ==================== VISION ALIGNMENT ====================
    // A button - Align to AprilTag (CENTER position)
    m_driverController
        .a()
        .whileTrue(new AlignToAprilTag(drivetrain, limelight, AlignPosition.CENTER));

    // Left Bumper - Align to AprilTag (LEFT position)
    m_driverController
        .leftBumper()
        .whileTrue(new AlignToAprilTag(drivetrain, limelight, AlignPosition.LEFT));

    // Right Bumper - Align to AprilTag (RIGHT position)
    m_driverController
        .rightBumper()
        .whileTrue(new AlignToAprilTag(drivetrain, limelight, AlignPosition.RIGHT));

    // Y button - Reset Heading
    m_driverController.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // ==================== SHOOTING CONTROLS ====================

    // X Button - Auto-Range and Spin Up Shooter
    // While held: Calculates distance, sets RPM, spins motor.
    // Release: Stops shooter.
    m_driverController
        .x()
        .whileTrue(new AutoShoot(
            shooter, limelight)); // X button(might conflict with other buttons so be aware)

    // Right Trigger - Run Indexer Forward (Intake/Feed)
    // NOTE: Driver holds X to spin up, then pulls Trigger to fire!
    m_driverController
        .rightTrigger(0.5)
        .whileTrue(new RunIndexer(indexer, Constants.IndexerConstants.kForwardSpeed)
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));

    // B Button - Run Indexer Reverse (Unjam)
    m_driverController
        .b()
        .whileTrue(new RunIndexer(indexer, Constants.IndexerConstants.kReverseSpeed)
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));

    // ==================== SLOW MODE ====================
    // Left trigger - Hold for slow mode (useful for precise positioning/scoring)
    m_driverController
        .leftTrigger(0.5)
        .whileTrue(Commands.startEnd(
            () -> {
              drivetrain.setTeleopVelocityCoefficient(
                  Constants.DrivetrainConstants.SLOW_MODE_COEFFICIENT);
              drivetrain.setRotationVelocityCoefficient(
                  Constants.DrivetrainConstants.SLOW_MODE_COEFFICIENT);
            },
            () -> {
              drivetrain.setTeleopVelocityCoefficient(
                  Constants.DrivetrainConstants.NORMAL_SPEED_COEFFICIENT);
              drivetrain.setRotationVelocityCoefficient(
                  Constants.DrivetrainConstants.NORMAL_SPEED_COEFFICIENT);
            }));
  }

  /** Get the driver controller for use in commands/subsystems */
  public CommandXboxController getDriverController() {
    return m_driverController;
  }

  /** Get the operator controller for use in commands/subsystems */
  public CommandXboxController getOperatorController() {
    return m_operatorController;
  }

  /** Get the state machine instance */
  public RobotStateMachine getStateMachine() {
    return m_stateMachine;
  }

  /**
   * Returns the autonomous command to run during autonomous period TODO: Implement autonomous
   * routines using PathPlanner
   */
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
