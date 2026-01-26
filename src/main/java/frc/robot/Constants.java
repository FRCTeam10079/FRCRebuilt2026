// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Constants for FRC 2026 REBUILT season Contains game-specific values, timing, and robot
 * configuration
 */
public final class Constants {

  /** Game timing constants for REBUILT */
  public static final class GameConstants {
    // Match period durations (seconds)
    public static final double AUTO_DURATION = 20.0; // 0:00 - 0:20
    public static final double TELEOP_DURATION = 150.0; // 2:30 total
    public static final double ENDGAME_THRESHOLD = 30.0; // Last 30 seconds

    // Hub Shift timing (transition period where both hubs are active)
    public static final double TRANSITION_START_TIME = 150.0; // Match time when transition starts
    public static final double TRANSITION_END_TIME = 140.0; // Match time when transition ends
    public static final double TRANSITION_DURATION = 10.0; // 10 second transition period

    // Fuel constants
    public static final int PRELOAD_FUEL_LIMIT = 8;
    // NO max capacity during gameplay - robots can hoard unlimited fuel!

    // Climb scoring
    public static final int CLIMB_L1_POINTS = 15; // Level 1 climb points
    public static final int CLIMB_L2_POINTS = 30; // Level 2 climb points (estimated)
    public static final int CLIMB_L3_POINTS = 50; // Level 3 climb points (estimated)

    // Height restriction during climb (inches)
    public static final double MAX_HEIGHT_DURING_CLIMB = 30.0;
  }

  /** Controller port assignments */
  public static final class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // Deadband for joystick inputs
    public static final double JOYSTICK_DEADBAND = 0.1;
  }

  /** Drivetrain constants */
  public static final class DrivetrainConstants {
    // ==================== SPEEDS ====================
    // These are the ABSOLUTE maximums - derived from TunerConstants.kSpeedAt12Volts
    // The robot physically cannot exceed these values
    public static final double MAX_SPEED_MPS =
        10.0; // The theoretical possible maximum translation speed is 10.81
    // m/s - from TunerConstants
    public static final double MAX_ANGULAR_RATE_RAD_PER_SEC = Math.PI * 2.0; // 360 deg/s
    // ==================== SPEED COEFFICIENTS ====================
    // Runtime multipliers applied to MAX_SPEED - these can be changed dynamically
    // Use: swerve.driveFieldCentricSmooth(..., MAX_SPEED_MPS *
    // NORMAL_SPEED_COEFFICIENT, ...)
    public static final double NORMAL_SPEED_COEFFICIENT = 1.0; // Full speed
    public static final double SLOW_MODE_COEFFICIENT = 0.7; // Precision mode (70%)
    public static final double SCORING_SPEED_COEFFICIENT = 0.5; // Slow for scoring

    // ==================== DEADBAND ====================
    public static final double DEADBAND_PERCENT = 0.1;

    // Vision alignment tolerances
    public static final double POSITION_TOLERANCE_METERS = 0.02; // 2cm position tolerance
    public static final double YAW_TOLERANCE_RADIANS = Math.PI / 32; // ~5.6 degrees

    // Alignment PID values
    public static final double ALIGN_PID_KP = 8.0;
    public static final double ALIGN_PID_KI = 0.0;
    public static final double ALIGN_PID_KD = 0.01;
    public static final double ALIGN_ROTATION_KP = 3.0;
    public static final double ALIGN_ROTATION_KD = 0.02;

    // Movement speeds during alignment (m/s, rad/s)
    public static final double ALIGN_SPEED_MPS = 3.5;
    public static final double ALIGN_ROTATION_SPEED = 0.9;

    // Alignment offset distances from AprilTag (meters)
    // These define where the robot stops relative to the tag
    public static final double ALIGN_OFFSET_X_LEFT = -0.41; // Back from tag for LEFT
    public static final double ALIGN_OFFSET_Y_LEFT = 0.13; // Left of tag center
    public static final double ALIGN_OFFSET_X_RIGHT = -0.41; // Back from tag for RIGHT
    public static final double ALIGN_OFFSET_Y_RIGHT = -0.23; // Right of tag center
    public static final double ALIGN_OFFSET_X_CENTER = -0.50; // Back from tag for CENTER
    public static final double ALIGN_OFFSET_Y_CENTER = 0.0; // Centered on tag
  }

  /** Intake constants (placeholder) */
  public static final class IntakeConstants {
    public static final double INTAKE_SPEED = 0.8;
    public static final double OUTTAKE_SPEED = -0.5;
    public static final int INTAKE_MOTOR_ID = 0; // TODO: Make real
    public static final int PIVOT_ENCODER_ID = 1; // TODO: Make real
    public static final int PIVOT_MOTOR_ID = 2; // TODO: Make real
    public static final double PIVOT_INTAKE_POSITION = 0.0; // TODO: Tune
    public static final double PIVOT_STOWED_POSITION = 0.5; // TODO: Tune
  }

  /** Shooter constants (placeholder) */
  public static final class ShooterConstants {
    // Target RPM values (will vary based on distance)
    public static final double SHOOTER_IDLE_RPM = 0;
    public static final double SHOOTER_SPINUP_RPM = 3000;
    public static final double SHOOTER_MAX_RPM = 5000;

    // RPM tolerance for "at setpoint" check
    public static final double SHOOTER_RPM_TOLERANCE = 100;

    // Feeder speed when firing
    public static final double FEEDER_SPEED = 1.0;
  }

  /** Climber constants (placeholder) */
  public static final class ClimberConstants {
    // Climber arm positions (encoder units - fill in with actual values later)
    public static final double CLIMBER_STOWED_POSITION = 0;
    public static final double CLIMBER_L1_REACH_POSITION = 50;
    public static final double CLIMBER_L2_REACH_POSITION = 80;
    public static final double CLIMBER_L3_REACH_POSITION = 100;

    // Climber motor speeds
    public static final double CLIMBER_EXTEND_SPEED = 0.7;
    public static final double CLIMBER_RETRACT_SPEED = -0.8;
  }

  /** State machine timing constants */
  public static final class StateMachineConstants {
    // Rumble feedback durations (seconds)
    public static final double RUMBLE_SHORT = 0.15;
    public static final double RUMBLE_MEDIUM = 0.3;
    public static final double RUMBLE_LONG = 0.5;
    public static final double RUMBLE_EXTRA_LONG = 1.0;

    // Rumble intensities
    public static final double RUMBLE_LIGHT = 0.3;
    public static final double RUMBLE_MEDIUM_INTENSITY = 0.5;
    public static final double RUMBLE_STRONG = 0.8;
    public static final double RUMBLE_MAX = 1.0;

    // State history buffer size
    public static final int STATE_HISTORY_SIZE = 20;

    /** Minimum cycle time to be considered valid (seconds) */
    public static final double MIN_VALID_CYCLE_TIME = 1.0;
  }

  /** Vision/Limelight constants (placeholder) */
  public static final class VisionConstants {
    public static final String LIMELIGHT_NAME = "limelight";

    // Pipeline IDs
    public static final int PIPELINE_HUB_TRACKING = 0;
    public static final int PIPELINE_FUEL_DETECTION = 1;
    public static final int PIPELINE_APRILTAG = 2;

    // Target height for hub (inches from floor)
    public static final double HUB_TARGET_HEIGHT_INCHES = 104.0; // Placeholder

    // Camera mounting (inches)
    public static final double CAMERA_HEIGHT_INCHES = 24.0; // Placeholder
    public static final double CAMERA_MOUNT_ANGLE_DEGREES = 30.0; // Placeholder
  }

  public static final class IndexerConstants {

    public static final int kFeederMotorID = 15; // This ID is a placeholder
    public static final int kSpindexerMotorID = 20; // This id confirmed

    // Safety Limits
    public static final int kCurrentLimit = 40;

    // Speeds (-1.0 to 1.0)
    public static final double kForwardSpeed = 0.5;
    public static final double kReverseSpeed = -0.5;
  }
  // ==================== UTILITY METHODS ====================

  /** Inches to Meters conversion factor */
  public static final double INCHES_TO_METERS = 0.0254;

  /**
   * Check if a value exists in an array
   *
   * @param array The array to search
   * @param value The value to find
   * @return True if value is in array
   */
  public static boolean contains(double[] array, double value) {
    for (double element : array) {
      if (element == value) {
        return true;
      }
    }
    return false;
  }

  /**
   * Check if an int value exists in an int array
   *
   * @param array The array to search
   * @param value The value to find
   * @return True if value is in array
   */
  public static boolean contains(int[] array, int value) {
    for (int element : array) {
      if (element == value) {
        return true;
      }
    }
    return false;
  }

  // ==================== APRIL TAG FIELD LAYOUT ====================

  /**
   * AprilTag positions on the field
   *
   * <p>NOTE: This is a PLACEHOLDER using 2025 Reefscape field layout! Replace with actual 2026
   * REBUILT field AprilTag positions when released.
   *
   * <p>Format: HashMap<TagID, double[]{X_inches, Y_inches, Z_inches, Yaw_degrees, Pitch_degrees}>
   */
  public static final class AprilTagMaps {
    public static final java.util.HashMap<Integer, double[]> aprilTagMap =
        new java.util.HashMap<>();

    static {
      // Points are in inches, Angles are in degrees
      // Format: {X, Y, Z, Yaw, Pitch}

      // RED SIDE
      aprilTagMap.put(3, new double[] {455.15, 317.15, 51.25, 270.0, 0.0});
      aprilTagMap.put(4, new double[] {365.20, 241.64, 73.54, 0.0, 30.0});
      aprilTagMap.put(5, new double[] {365.20, 75.39, 73.54, 0.0, 30.0});
      aprilTagMap.put(6, new double[] {530.49, 130.17, 12.13, 300.0, 0.0});
      aprilTagMap.put(7, new double[] {546.87, 158.50, 12.13, 0.0, 0.0});
      aprilTagMap.put(8, new double[] {530.49, 186.83, 12.13, 60.0, 0.0});
      aprilTagMap.put(9, new double[] {497.77, 186.83, 12.13, 120.0, 0.0});
      aprilTagMap.put(10, new double[] {481.39, 158.50, 12.13, 180.0, 0.0});
      aprilTagMap.put(11, new double[] {497.77, 130.17, 12.13, 240.0, 0.0});

      // BLUE SIDE
      aprilTagMap.put(14, new double[] {325.68, 241.64, 73.54, 180.0, 30.0});
      aprilTagMap.put(15, new double[] {325.68, 75.39, 73.54, 180.0, 30.0});
      aprilTagMap.put(16, new double[] {235.73, -0.15, 51.25, 90.0, 0.0});
      aprilTagMap.put(17, new double[] {160.39, 130.17, 12.13, 240.0, 0.0});
      aprilTagMap.put(18, new double[] {144.00, 158.50, 12.13, 180.0, 0.0});
      aprilTagMap.put(19, new double[] {160.39, 186.83, 12.13, 120.0, 0.0});
      aprilTagMap.put(20, new double[] {193.10, 186.83, 12.13, 60.0, 0.0});
      aprilTagMap.put(21, new double[] {209.49, 158.50, 12.13, 0.0, 0.0});
      aprilTagMap.put(22, new double[] {193.10, 130.17, 12.13, 300.0, 0.0});
    }

    // Red side tag IDs (for direction flipping logic)
    public static final int[] RED_SIDE_TAGS = {3, 4, 5, 6, 7, 8, 9, 10, 11};

    // Blue side tag IDs
    public static final int[] BLUE_SIDE_TAGS = {14, 15, 16, 17, 18, 19, 20, 21, 22};
  }

  /** Alignment position enum - left or right side offset from AprilTag */
  public enum AlignPosition {
    LEFT,
    RIGHT,
    CENTER
  }

  /** Starting position enum for autonomous */
  public enum StartingPosition {
    LEFT,
    CENTER,
    RIGHT
  }
}
