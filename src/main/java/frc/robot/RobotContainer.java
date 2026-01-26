// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AlignPosition;
import frc.robot.commands.AlignToAprilTag;
import frc.robot.generated.TunerConstants;
import frc.robot.pathfinding.Pathfinding;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

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

  public RobotContainer() {
    // Link limelight to drivetrain for vision-based odometry
    limelight.setDrivetrain(drivetrain);

    // Register controllers with state machine for haptic feedback
    m_stateMachine.registerControllers(m_driverController, m_operatorController);

    // Initialize the pathfinding system
    initializePathfinding();

    // Configure button bindings
    configureBindings();
  }

  /**
   * Initialize the pathfinding system. This loads the navgrid and starts the background AD*
   * planning thread.
   */
  private void initializePathfinding() {
    System.out.println("[RobotContainer] Initializing pathfinding system...");
    Pathfinding.ensureInitialized();
    System.out.println("[RobotContainer] Pathfinding system ready");
  }

  /**
   * Configure button bindings for driver and operator controllers This is where you bind controller
   * buttons to commands
   */
  private void configureBindings() {
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

    // ==================== OPERATOR CONTROLS ====================
    // TODO: Add intake controls
    // TODO: Add shooter controls
    // TODO: Add climb controls

    // ==================== PATHFINDING CONTROLS ====================
    // X button - Pathfind to AprilTag 10 (Red Alliance Hub Face)
    // Uses AD* algorithm to find safe path around obstacles
    m_driverController.x().whileTrue(drivetrain.pathfindToAprilTag10());

    // B button - Pathfind to AprilTag 18 (Blue Alliance Hub Face)
    // Alternative hub for testing/blue alliance
    m_driverController.b().whileTrue(drivetrain.pathfindToAprilTag(18));

    // ==================== STATE MACHINE EXAMPLES ====================
    // Example: Manual state transitions (add your actual bindings)
    // m_driverController.y().onTrue(Commands.runOnce(() ->
    //     m_stateMachine.setGameState(RobotStateMachine.GameState.AIMING_AT_HUB)));

    // Example: Hub shift state can be set based on FMS data or operator input
    // m_operatorController.start().onTrue(Commands.runOnce(() ->
    //     m_stateMachine.setHubShiftState(RobotStateMachine.HubShiftState.MY_HUB_ACTIVE)));
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
   * Returns the autonomous command to run during autonomous period. Uses pathfinding to navigate to
   * AprilTag 10 (Red Alliance Hub).
   */
  public Command getAutonomousCommand() {
    // Set pathfinding to use autonomous obstacles (tighter margins)
    Pathfinding.setAutoObstacles();

    // Return pathfinding command to AprilTag 10
    return drivetrain.pathfindToAprilTag10().withName("Auto: Pathfind to Tag 10");
  }
}
