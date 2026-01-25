// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Defines all LED states for the robot with their visual patterns. Each state has a priority level
 * - higher priority states override lower ones.
 *
 * <p>Priority Levels: 0=Idle, 1=Match state, 2=Game piece, 3=Actions, 4=Alignment, 5=Alerts,
 * 6=Endgame/Climb, 7=Emergency
 */
public enum LEDState {
  // ==================== DEFAULT/IDLE (Priority 0) ====================
  OFF("Off", 0, LEDPattern.solid(Color.kBlack)),
  IDLE_RAINBOW(
      "Idle Rainbow",
      0,
      LEDPattern.rainbow(255, 128).scrollAtRelativeSpeed(Percent.per(Second).of(50))),

  // ==================== MATCH STATE (Priority 1) ====================
  DISABLED_BLUE(
      "Disabled (Blue Alliance)", 1, LEDPattern.solid(Color.kBlue).breathe(Seconds.of(2.0))),
  DISABLED_RED("Disabled (Red Alliance)", 1, LEDPattern.solid(Color.kRed).breathe(Seconds.of(2.0))),
  DISABLED_UNKNOWN(
      "Disabled (Unknown)", 1, LEDPattern.solid(Color.kPurple).breathe(Seconds.of(2.0))),

  AUTO_RUNNING("Auto Running", 1, LEDPattern.solid(Color.kGold).blink(Seconds.of(0.5))),
  TELEOP_RUNNING("Teleop Running", 1, LEDPattern.solid(Color.kGreen)),

  // ==================== HUB SHIFT STATE (Priority 1) ====================
  HUB_ACTIVE("Hub Active - SCORE!", 1, LEDPattern.solid(Color.kLime)),
  HUB_INACTIVE("Hub Inactive - HOARD", 1, LEDPattern.solid(Color.kOrange).breathe(Seconds.of(1.5))),
  HUB_TRANSITION(
      "Transition Period",
      1,
      LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGreen, Color.kYellow)
          .scrollAtRelativeSpeed(Percent.per(Second).of(100))),

  // ==================== GAME PIECE STATUS (Priority 2) ====================
  FUEL_EMPTY("No Fuel", 2, LEDPattern.solid(Color.kDarkRed)),
  FUEL_ACQUIRING("Acquiring Fuel", 2, LEDPattern.solid(Color.kYellow).blink(Seconds.of(0.15))),
  FUEL_LOADED("Fuel Loaded", 2, LEDPattern.solid(Color.kGreen)),
  FUEL_FULL(
      "Max Fuel",
      2,
      LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGreen, Color.kLime)
          .scrollAtRelativeSpeed(Percent.per(Second).of(25))),

  // ==================== ACTION IN PROGRESS (Priority 3) ====================
  INTAKING("Intaking", 3, LEDPattern.solid(Color.kOrange).blink(Seconds.of(0.1))),
  INTAKE_SUCCESS(
      "Intake Success",
      3,
      LEDPattern.solid(Color.kGreen).blink(Seconds.of(0.08), Seconds.of(0.08))),
  OUTTAKING("Outtaking", 3, LEDPattern.solid(Color.kRed).blink(Seconds.of(0.2))),
  SHOOTER_SPINUP("Shooter Spinning Up", 3, LEDPattern.solid(Color.kYellow).blink(Seconds.of(0.25))),
  SHOOTER_READY("Shooter At Speed", 3, LEDPattern.solid(Color.kLime)),
  FIRING("Firing!", 3, LEDPattern.solid(Color.kWhite).blink(Seconds.of(0.05))),

  // ==================== ALIGNMENT/TARGETING (Priority 4) ====================
  ALIGNING("Aligning to Target", 4, LEDPattern.solid(Color.kCyan).blink(Seconds.of(0.2))),
  ALIGNED("Target Aligned", 4, LEDPattern.solid(Color.kGreen)),
  NO_TARGET("No Target Visible", 4, LEDPattern.solid(Color.kRed).blink(Seconds.of(0.5))),
  VISION_TRACKING("Vision Tracking", 4, LEDPattern.solid(Color.kCyan)),

  // ==================== CRITICAL ALERTS (Priority 5) ====================
  LOW_BATTERY("Low Battery", 5, LEDPattern.solid(Color.kRed).blink(Seconds.of(0.25))),
  BROWNOUT_WARNING(
      "Brownout Warning", 5, LEDPattern.solid(Color.kOrangeRed).blink(Seconds.of(0.1))),
  SENSOR_ERROR(
      "Sensor Error",
      5,
      LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed, Color.kYellow)
          .blink(Seconds.of(0.3))),
  CAN_ERROR("CAN Bus Error", 5, LEDPattern.solid(Color.kPurple).blink(Seconds.of(0.2))),

  // ==================== ENDGAME/CLIMB (Priority 6) ====================
  ENDGAME_WARNING("Endgame Soon!", 6, LEDPattern.solid(Color.kYellow).blink(Seconds.of(0.5))),
  APPROACHING_CLIMB("Approaching Climb", 6, LEDPattern.solid(Color.kPurple)),
  CLIMBING_L1("Climbing Level 1", 6, LEDPattern.solid(Color.kBlue).blink(Seconds.of(0.3))),
  CLIMBING_L2(
      "Climbing Level 2",
      6,
      LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kBlue, Color.kCyan)
          .scrollAtRelativeSpeed(Percent.per(Second).of(50))),
  CLIMBING_L3(
      "Climbing Level 3",
      6,
      LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kCyan, Color.kWhite)
          .scrollAtRelativeSpeed(Percent.per(Second).of(75))),
  CLIMB_SUCCESS(
      "Climb Success!",
      6,
      LEDPattern.rainbow(255, 200).scrollAtRelativeSpeed(Percent.per(Second).of(100))),
  CLIMB_FAILED("Climb Failed", 6, LEDPattern.solid(Color.kRed).blink(Seconds.of(0.1))),
  HEIGHT_CHECK("Height Check", 6, LEDPattern.solid(Color.kYellow).blink(Seconds.of(0.15))),

  // ==================== EMERGENCY/OVERRIDE (Priority 7) ====================
  ESTOP("Emergency Stop", 7, LEDPattern.solid(Color.kRed)),
  MANUAL_OVERRIDE("Manual Override", 7, LEDPattern.solid(Color.kWhite).blink(Seconds.of(0.5))),
  BRAKE_MODE("Brake Mode Active", 7, LEDPattern.solid(Color.kOrange)),
  COAST_MODE("Coast Mode Active", 7, LEDPattern.solid(Color.kAqua).blink(Seconds.of(0.5)));

  private final String name;
  private final int priority;
  private final LEDPattern pattern;

  LEDState(String name, int priority, LEDPattern pattern) {
    this.name = name;
    this.priority = priority;
    this.pattern = pattern;
  }

  /** @return The display name of this state */
  public String getName() {
    return name;
  }

  /** @return The priority level (0-7) */
  public int getPriority() {
    return priority;
  }

  /** @return The LED pattern for this state */
  public LEDPattern getPattern() {
    return pattern;
  }

  /** @return true if this state has higher or equal priority than the other */
  public boolean hasHigherOrEqualPriority(LEDState other) {
    return this.priority >= other.priority;
  }

  @Override
  public String toString() {
    return name + " (P" + priority + ")";
  }
}
