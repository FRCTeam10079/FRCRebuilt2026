// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStateMachine;
import frc.robot.RobotStateMachine.AllianceColor;
import frc.robot.RobotStateMachine.ClimbState;
import frc.robot.RobotStateMachine.FuelState;
import frc.robot.RobotStateMachine.GameState;
import frc.robot.RobotStateMachine.HubShiftState;
import frc.robot.RobotStateMachine.MatchState;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * LED Subsystem for state-dependent lighting control. Uses a priority-based system where
 * higher-priority states override lower ones. Supports automatic state selection from the robot
 * state machine, manual overrides via commands, and full simulation support via Glass.
 */
public class LEDSubsystem extends SubsystemBase {

  // IO layer for hardware abstraction
  private final LEDIO io;

  // Reference to robot state machine
  private final RobotStateMachine stateMachine;

  // Current and requested states
  private LEDState currentState = LEDState.OFF;
  private LEDState requestedState = LEDState.OFF;

  // Override state (set via commands, cleared when command ends)
  private LEDState overrideState = null;

  // Flash state tracking
  private LEDState flashState = null;
  private double flashEndTime = 0;

  // External state suppliers (for alignment, shooter ready, etc.)
  private BooleanSupplier alignedSupplier = () -> false;
  private BooleanSupplier targetVisibleSupplier = () -> false;
  private BooleanSupplier shooterReadySupplier = () -> false;
  private BooleanSupplier intakingSupplier = () -> false;
  private DoubleSupplier batteryVoltageSupplier = RobotController::getBatteryVoltage;

  // Timing
  private double lastUpdateTime = 0;

  /** Creates the LED subsystem with automatic hardware detection (sim vs real). */
  public LEDSubsystem(RobotStateMachine stateMachine) {
    this(
        stateMachine,
        RobotBase.isSimulation()
            ? new LEDIOSim(LEDConstants.LED_COUNT)
            : new LEDIOHardware(LEDConstants.LED_PWM_PORT, LEDConstants.LED_COUNT));
  }

  /** Creates the LED subsystem with a specified IO implementation. */
  public LEDSubsystem(RobotStateMachine stateMachine, LEDIO io) {
    this.stateMachine = stateMachine;
    this.io = io;

    // Set default command to auto-select state based on robot state
    setDefaultCommand(runAutoStateSelection().withName("LED Auto State"));
  }

  @Override
  public void periodic() {
    double currentTime = Timer.getFPGATimestamp();

    // Clear flash state if expired
    if (flashState != null && currentTime >= flashEndTime) {
      flashState = null;
    }

    // Determine which state to display (priority: flash > override > requested)
    LEDState stateToApply;
    if (flashState != null) {
      stateToApply = flashState;
    } else if (overrideState != null) {
      stateToApply = overrideState;
    } else {
      stateToApply = requestedState;
    }

    // Only update if state changed or enough time has passed
    if (stateToApply != currentState
        || (currentTime - lastUpdateTime) >= LEDConstants.LED_UPDATE_PERIOD) {
      currentState = stateToApply;
      lastUpdateTime = currentTime;

      // Apply the pattern to the buffer
      currentState.getPattern().applyTo(io.getBuffer());

      // Update simulation display if applicable
      if (io instanceof LEDIOSim) {
        ((LEDIOSim) io).setStateName(currentState.getName());
      }
    }

    // Always update IO (sends buffer data to hardware/simulation)
    io.update();

    // Update telemetry
    updateTelemetry();
  }

  /** Updates SmartDashboard with LED state information. */
  private void updateTelemetry() {
    SmartDashboard.putString("LED/Current State", currentState.getName());
    SmartDashboard.putNumber("LED/State Priority", currentState.getPriority());
    SmartDashboard.putBoolean("LED/Has Override", overrideState != null);
    SmartDashboard.putBoolean("LED/Is Flashing", flashState != null);
    SmartDashboard.putBoolean("LED/Is Simulation", io.isSimulation());
  }

  // ==================== STATE SELECTION LOGIC ====================

  /** Determines the appropriate LED state based on the current robot state. */
  private LEDState determineStateFromRobot() {
    // Check for critical alerts first (highest non-emergency priority)
    double voltage = batteryVoltageSupplier.getAsDouble();
    if (voltage < LEDConstants.BROWNOUT_WARNING_VOLTAGE) {
      return LEDState.BROWNOUT_WARNING;
    }
    if (voltage < LEDConstants.LOW_BATTERY_VOLTAGE) {
      return LEDState.LOW_BATTERY;
    }

    // Get current states from state machine
    MatchState matchState = stateMachine.getMatchState();
    GameState gameState = stateMachine.getGameState();
    FuelState fuelState = stateMachine.getFuelState();
    HubShiftState hubShiftState = stateMachine.getHubShiftState();
    ClimbState climbState = stateMachine.getClimbState();
    AllianceColor alliance = stateMachine.getAlliance();

    // Handle disabled state
    if (!DriverStation.isEnabled()) {
      return switch (alliance) {
        case RED -> LEDState.DISABLED_RED;
        case BLUE -> LEDState.DISABLED_BLUE;
        default -> LEDState.DISABLED_UNKNOWN;
      };
    }

    // Handle emergency stop
    if (matchState == MatchState.ESTOP || gameState == GameState.EMERGENCY_STOP) {
      return LEDState.ESTOP;
    }

    // Handle climb states (priority 6)
    if (climbState.isClimbing()) {
      return switch (climbState) {
        case DEPLOYING_L1, PULLING_UP_L1 -> LEDState.CLIMBING_L1;
        case REACHING_L2, GRABBING_L2 -> LEDState.CLIMBING_L2;
        case REACHING_L3, GRABBING_L3 -> LEDState.CLIMBING_L3;
        case HEIGHT_CHECK_L2, HEIGHT_CHECK_L3 -> LEDState.HEIGHT_CHECK;
        case ENGAGED_BRAKE -> LEDState.CLIMB_SUCCESS;
        case CLIMB_FAILED -> LEDState.CLIMB_FAILED;
        default -> LEDState.APPROACHING_CLIMB;
      };
    }

    if (climbState.isCompleted()) {
      return LEDState.CLIMB_SUCCESS;
    }

    // Handle endgame warning
    if (gameState.isEndgame() && !climbState.isClimbing()) {
      return LEDState.ENDGAME_WARNING;
    }

    // Handle alignment states (priority 4)
    if (targetVisibleSupplier.getAsBoolean()) {
      if (alignedSupplier.getAsBoolean()) {
        return LEDState.ALIGNED;
      } else {
        return LEDState.ALIGNING;
      }
    }

    // Handle shooter states (priority 3)
    if (gameState == GameState.FIRING_FUEL) {
      return LEDState.FIRING;
    }
    if (gameState == GameState.SHOOTER_SPINUP) {
      if (shooterReadySupplier.getAsBoolean()) {
        return LEDState.SHOOTER_READY;
      }
      return LEDState.SHOOTER_SPINUP;
    }

    // Handle intake states (priority 3)
    if (intakingSupplier.getAsBoolean()
        || gameState == GameState.COLLECTING_GROUND
        || gameState == GameState.AUTO_COLLECTING
        || fuelState == FuelState.ACQUIRING) {
      return LEDState.INTAKING;
    }

    // Handle fuel status (priority 2)
    int fuelCount = stateMachine.getFuelCount();
    if (fuelCount >= LEDConstants.FUEL_FULL_THRESHOLD) {
      return LEDState.FUEL_FULL;
    }
    if (fuelState == FuelState.LOADED || fuelCount > 0) {
      return LEDState.FUEL_LOADED;
    }
    if (fuelState == FuelState.EMPTY) {
      return LEDState.FUEL_EMPTY;
    }

    // Handle hub shift states (priority 1)
    return switch (hubShiftState) {
      case MY_HUB_ACTIVE -> LEDState.HUB_ACTIVE;
      case MY_HUB_INACTIVE -> LEDState.HUB_INACTIVE;
      case TRANSITION -> LEDState.HUB_TRANSITION;
      default -> {
        // Handle match state
        if (matchState == MatchState.AUTO_RUNNING) {
          yield LEDState.AUTO_RUNNING;
        } else if (matchState == MatchState.TELEOP_RUNNING) {
          yield LEDState.TELEOP_RUNNING;
        }
        yield LEDState.IDLE_RAINBOW;
      }
    };
  }

  // ==================== COMMAND FACTORIES ====================

  /** Creates a command that automatically selects LED state based on robot state. */
  public Command runAutoStateSelection() {
    return run(() -> {
          overrideState = null; // Clear any override
          requestedState = determineStateFromRobot();
        })
        .withName("LED Auto");
  }

  /** Creates a command that sets a specific LED state until the command ends. */
  public Command runState(LEDState state) {
    return run(() -> overrideState = state)
        .finallyDo(() -> overrideState = null)
        .ignoringDisable(true)
        .withName("LED: " + state.getName());
  }

  /** Creates a command that sets LED state based on a supplier for dynamic selection. */
  public Command runState(Supplier<LEDState> stateSupplier) {
    return run(() -> overrideState = stateSupplier.get())
        .finallyDo(() -> overrideState = null)
        .ignoringDisable(true)
        .withName("LED Dynamic");
  }

  /** Creates a command that blinks between two states. */
  public Command runBlinking(
      LEDState stateOne, LEDState stateTwo, double onDuration, double offDuration) {
    return Commands.sequence(
            runState(stateOne).withTimeout(onDuration), runState(stateTwo).withTimeout(offDuration))
        .repeatedly()
        .ignoringDisable(true)
        .withName("LED Blink");
  }

  /** Triggers a flash effect that temporarily overrides the current state. */
  public Command flash(LEDState state, double duration) {
    return Commands.runOnce(
            () -> {
              flashState = state;
              flashEndTime = Timer.getFPGATimestamp() + duration;
            },
            this)
        .withName("LED Flash: " + state.getName());
  }

  /** Flashes a quick success indication. */
  public Command flashSuccess() {
    return flash(LEDState.INTAKE_SUCCESS, LEDConstants.FLASH_DURATION_SECONDS);
  }

  /** Flashes a quick aligned indication. */
  public Command flashAligned() {
    return flash(LEDState.ALIGNED, LEDConstants.FLASH_DURATION_SECONDS);
  }

  // ==================== SUPPLIER CONFIGURATION ====================

  /** Sets the supplier for alignment status (true when aligned to target). */
  public void setAlignedSupplier(BooleanSupplier supplier) {
    this.alignedSupplier = supplier;
  }

  /** Sets the supplier for target visibility (true when vision target is visible). */
  public void setTargetVisibleSupplier(BooleanSupplier supplier) {
    this.targetVisibleSupplier = supplier;
  }

  /** Sets the supplier for shooter ready status (true when at target RPM). */
  public void setShooterReadySupplier(BooleanSupplier supplier) {
    this.shooterReadySupplier = supplier;
  }

  /** Sets the supplier for intake active status (true when intake is running). */
  public void setIntakingSupplier(BooleanSupplier supplier) {
    this.intakingSupplier = supplier;
  }

  /** Sets the supplier for battery voltage. */
  public void setBatteryVoltageSupplier(DoubleSupplier supplier) {
    this.batteryVoltageSupplier = supplier;
  }

  // ==================== GETTERS ====================

  /** @return The current LED state being displayed */
  public LEDState getCurrentState() {
    return currentState;
  }

  /** @return true if an override state is active */
  public boolean hasOverride() {
    return overrideState != null;
  }

  /** @return true if a flash effect is active */
  public boolean isFlashing() {
    return flashState != null;
  }

  /** @return The LED IO implementation */
  public LEDIO getIO() {
    return io;
  }
}
