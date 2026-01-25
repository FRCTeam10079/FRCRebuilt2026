// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

/** Constants for the LED subsystem. */
public final class LEDConstants {

  private LEDConstants() {
    // Utility class - no instantiation
  }

  // ==================== HARDWARE CONFIGURATION ====================

  /** PWM port for the AddressableLED data line */
  public static final int LED_PWM_PORT = 9;

  /** Total number of LEDs in the strip */
  public static final int LED_COUNT = 60;

  /** Number of LEDs on the CANdle */
  public static final int CANDLE_LED_COUNT = 8;

  /** LED strip brightness (0.0 to 1.0) */
  public static final double DEFAULT_BRIGHTNESS = 0.8;

  // ==================== BATTERY THRESHOLDS ====================

  /** Battery voltage threshold for low battery warning */
  public static final double LOW_BATTERY_VOLTAGE = 11.5;

  /** Battery voltage threshold for brownout warning */
  public static final double BROWNOUT_WARNING_VOLTAGE = 10.5;

  // ==================== TIMING CONSTANTS ====================

  /** Endgame warning starts at this many seconds remaining */
  public static final double ENDGAME_WARNING_SECONDS = 30.0;

  /** Duration for flash effects (seconds) */
  public static final double FLASH_DURATION_SECONDS = 0.6;

  /** How often to update LED state (seconds) - runs every cycle by default */
  public static final double LED_UPDATE_PERIOD = 0.02; // 50Hz

  // ==================== FUEL THRESHOLDS ====================

  /** Fuel count threshold for "full" indication */
  public static final int FUEL_FULL_THRESHOLD = 8;

  // ==================== STRIP SECTIONS (for multi-section strips) ====================
  // These can be used with AddressableLEDBufferView for controlling different parts

  /** Start index for front LEDs */
  public static final int FRONT_START = 0;
  /** Number of front LEDs */
  public static final int FRONT_COUNT = 15;

  /** Start index for left side LEDs */
  public static final int LEFT_START = 15;
  /** Number of left side LEDs */
  public static final int LEFT_COUNT = 15;

  /** Start index for back LEDs */
  public static final int BACK_START = 30;
  /** Number of back LEDs */
  public static final int BACK_COUNT = 15;

  /** Start index for right side LEDs */
  public static final int RIGHT_START = 45;
  /** Number of right side LEDs */
  public static final int RIGHT_COUNT = 15;
}
