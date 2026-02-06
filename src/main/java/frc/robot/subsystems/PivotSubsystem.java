package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * TODO:
 *
 * <ul>
 *   <li>Add functions to move pivot
 *   <li>Add command factories
 * </ul>
 */
public class PivotSubsystem extends SubsystemBase {
  private final TalonFX motor = new TalonFX(Constants.IntakeConstants.PIVOT_MOTOR_ID);
  private final CANcoder encoder = new CANcoder(Constants.IntakeConstants.PIVOT_ENCODER_ID);
  private final PositionVoltage pivotPositionControl = new PositionVoltage(0).withSlot(0);
  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withSlot(0);

  public PivotSubsystem() {
    configureMotors();
  }

  void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.GravityType =
        com.ctre.phoenix6.signals.GravityTypeValue.Arm_Cosine; // For pivoting arms

    MotionMagicConfigs mm = config.MotionMagic;
    mm.withMotionMagicCruiseVelocity(
            RotationsPerSecond.of(0.5)) // 5 (mechanism) rotations per second cruise
        .withMotionMagicAcceleration(
            RotationsPerSecondPerSecond.of(1)) // Take approximately 0.5 seconds to reach max vel
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(0));

    Slot0Configs slot0 = config.Slot0;
    slot0.kS = 0.35; // Add 0.25 V output to overcome static friction
    slot0.kV = 12; // A velocity target of 1 rps results in 0.12 V output
    slot0.kG = 0; // No gravity
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 0.02; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output

    // Use the remote CANcoder for absolute position feedback
    config.Feedback.FeedbackRemoteSensorID = Constants.IntakeConstants.PIVOT_ENCODER_ID;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.Feedback.SensorToMechanismRatio = 1.0; // Adjust if there's gearing
    config.Feedback.RotorToSensorRatio = 81.2; // Not really sure if needed but included anyways

    // Current limits for pivot motor
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Limits to prevent over-rotation
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Constants.IntakeConstants.PIVOT_STOWED_POSITION; // Slightly past intake position
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Constants.IntakeConstants.PIVOT_INTAKE_POSITION; // Slightly past stowed
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    motor.getConfigurator().apply(config);

    // Set position update frequency for better feedback
    motor.getPosition().setUpdateFrequency(100);
    encoder.getPosition().setUpdateFrequency(100);
    motor.optimizeBusUtilization();
  }
}
