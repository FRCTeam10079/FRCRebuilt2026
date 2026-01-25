// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStateMachine;
import frc.robot.RobotStateMachine.GameState;
import java.util.function.BooleanSupplier;

/** Factory class for creating LED-related commands. */
public final class LEDFactory {

  private LEDFactory() {
    // Utility class - no instantiation
  }

  // ==================== INTAKE COMMANDS ====================

  /** Shows intaking state with success flash on game piece acquisition. */
  public static Command intakingWithFeedback(LEDSubsystem leds, BooleanSupplier hasGamePiece) {
    return leds.runState(LEDState.INTAKING)
        .until(hasGamePiece)
        .andThen(leds.flash(LEDState.INTAKE_SUCCESS, 0.5))
        .withName("LED Intake w/ Feedback");
  }

  /** Shows outtaking state. */
  public static Command outtaking(LEDSubsystem leds) {
    return leds.runState(LEDState.OUTTAKING).withName("LED Outtaking");
  }

  // ==================== SHOOTER COMMANDS ====================

  /** Shows shooter spinup progress and ready state. */
  public static Command shooterStatus(LEDSubsystem leds, BooleanSupplier isAtSpeed) {
    return leds.runState(
            () -> isAtSpeed.getAsBoolean() ? LEDState.SHOOTER_READY : LEDState.SHOOTER_SPINUP)
        .withName("LED Shooter Status");
  }

  /** Shows firing state. */
  public static Command firing(LEDSubsystem leds) {
    return leds.runState(LEDState.FIRING).withName("LED Firing");
  }

  // ==================== ALIGNMENT COMMANDS ====================

  /** Shows alignment status based on vision target visibility and alignment. */
  public static Command alignmentStatus(
      LEDSubsystem leds, BooleanSupplier hasTarget, BooleanSupplier isAligned) {
    return leds.runState(() -> {
          if (!hasTarget.getAsBoolean()) {
            return LEDState.NO_TARGET;
          }
          return isAligned.getAsBoolean() ? LEDState.ALIGNED : LEDState.ALIGNING;
        })
        .withName("LED Alignment Status");
  }

  /** Shows aligning state and flashes when aligned. */
  public static Command aligningWithFlash(LEDSubsystem leds, BooleanSupplier isAligned) {
    return leds.runState(LEDState.ALIGNING)
        .until(isAligned)
        .andThen(leds.flash(LEDState.ALIGNED, 0.5))
        .withName("LED Aligning w/ Flash");
  }

  // ==================== CLIMB COMMANDS ====================

  /** Shows climb progress based on state machine. */
  public static Command climbStatus(LEDSubsystem leds, RobotStateMachine stateMachine) {
    return leds.runState(() -> {
          var climbState = stateMachine.getClimbState();
          return switch (climbState) {
            case DEPLOYING_L1, PULLING_UP_L1 -> LEDState.CLIMBING_L1;
            case REACHING_L2, GRABBING_L2 -> LEDState.CLIMBING_L2;
            case REACHING_L3, GRABBING_L3 -> LEDState.CLIMBING_L3;
            case HEIGHT_CHECK_L2, HEIGHT_CHECK_L3 -> LEDState.HEIGHT_CHECK;
            case ENGAGED_BRAKE -> LEDState.CLIMB_SUCCESS;
            case CLIMB_FAILED -> LEDState.CLIMB_FAILED;
            default -> LEDState.APPROACHING_CLIMB;
          };
        })
        .withName("LED Climb Status");
  }

  /** Shows rainbow celebration for successful climb. */
  public static Command climbCelebration(LEDSubsystem leds) {
    return leds.runState(LEDState.CLIMB_SUCCESS).withName("LED Climb Celebration");
  }

  // ==================== MODE INDICATOR COMMANDS ====================

  /** Shows game mode based on state machine (scoring, collecting, endgame, hub shift). */
  public static Command gameModeIndicator(LEDSubsystem leds, RobotStateMachine stateMachine) {
    return leds.runState(() -> {
          GameState state = stateMachine.getGameState();

          if (state.isScoring()) {
            return LEDState.FIRING;
          }
          if (state.isCollecting()) {
            return LEDState.INTAKING;
          }
          if (state.isEndgame()) {
            return LEDState.ENDGAME_WARNING;
          }

          // Default to hub shift indicator
          return switch (stateMachine.getHubShiftState()) {
            case MY_HUB_ACTIVE -> LEDState.HUB_ACTIVE;
            case MY_HUB_INACTIVE -> LEDState.HUB_INACTIVE;
            case TRANSITION -> LEDState.HUB_TRANSITION;
            default -> LEDState.TELEOP_RUNNING;
          };
        })
        .withName("LED Game Mode");
  }

  // ==================== UTILITY COMMANDS ====================

  /** Creates a blinking command between two states with the given period. */
  public static Command blink(
      LEDSubsystem leds, LEDState stateOne, LEDState stateTwo, double period) {
    return leds.runBlinking(stateOne, stateTwo, period / 2, period / 2);
  }

  /** Shows low battery warning. */
  public static Command lowBatteryWarning(LEDSubsystem leds) {
    return leds.runState(LEDState.LOW_BATTERY).withName("LED Low Battery");
  }

  /** Creates a conditional command that switches states based on a boolean supplier. */
  public static Command conditional(
      LEDSubsystem leds, BooleanSupplier condition, LEDState trueState, LEDState falseState) {
    return leds.runState(() -> condition.getAsBoolean() ? trueState : falseState)
        .withName("LED Conditional");
  }
}
