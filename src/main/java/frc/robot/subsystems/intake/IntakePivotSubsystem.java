package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

public class IntakePivotSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID);
    private final CANcoder encoder = new CANcoder(IntakeConstants.PIVOT_ENCODER_ID);
    private final Supplier<Angle> encoderAngle = encoder.getPosition().asSupplier();
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withSlot(0);

    private Angle setpoint = Radians.of(0.0);

    public IntakePivotSubsystem() {
        configureMotors();
    }

    void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // For pivoting arms

        config.MotionMagic.withMotionMagicCruiseVelocity(
                        IntakeConstants.PIVOT_VELOCITY) // 5 (mechanism) rotations per second cruise
                .withMotionMagicAcceleration(
                        IntakeConstants.PIVOT_ACCELERATION) // Take approximately 0.5 seconds to reach max vel
                .withMotionMagicJerk(IntakeConstants.PIVOT_JERK);

        config.Slot0.kS = IntakeConstants.PIVOT_KS; // Add 0.25 V output to overcome static friction
        config.Slot0.kV = IntakeConstants.PIVOT_KV; // A velocity target of 1 rps results in 0.12 V output
        config.Slot0.kG = IntakeConstants.PIVOT_KG; // No gravity
        config.Slot0.kA = IntakeConstants.PIVOT_KA; // An acceleration of 1 rps/s requires 0.01 V output
        config.Slot0.kP = IntakeConstants.PIVOT_KP; // A position error of 0.2 rotations results in 12 V output
        config.Slot0.kI = IntakeConstants.PIVOT_KI; // No output for integrated error
        config.Slot0.kD = IntakeConstants.PIVOT_KD; // A velocity error of 1 rps results in 0.5 V output

        // Use the remote CANcoder for absolute position feedback
        config.Feedback.FeedbackRemoteSensorID = IntakeConstants.PIVOT_ENCODER_ID;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        // Current limits for pivot motor
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.PIVOT_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Limits to prevent over-rotation
        config.SoftwareLimitSwitch.withForwardSoftLimitThreshold(IntakeConstants.PIVOT_STOWED_POSITION);
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.withReverseSoftLimitThreshold(IntakeConstants.PIVOT_INTAKE_POSITION);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        motor.getConfigurator().apply(config);

        // Set position update frequency for better feedback
        motor.getPosition().setUpdateFrequency(ENCODER_POLLING_RATE);
        encoder.getPosition().setUpdateFrequency(ENCODER_POLLING_RATE);
        motor.optimizeBusUtilization();
    }

    public boolean isAtSetpoint(Angle tolerance) {
        return encoderAngle.get().isNear(setpoint, tolerance);
    }

    public boolean isAtSetpoint() {
        return isAtSetpoint(Degrees.of(10.0));
    }

    public void setPivotSetpoint(Angle setpoint) {
        this.setpoint = setpoint;
    }

    /** Set the pivot setpoint then immediately continue. */
    public Command setPivotCommand(Supplier<Angle> setpoint) {
        return Commands.runOnce(() -> setPivotSetpoint(setpoint.get()), this);
    }

    /** Pivot to setpoint while waiting for the pivot to reach the setpoint. */
    public Command pivotToCommand(Supplier<Angle> setpoint) {
        return Commands.run(() -> setPivotSetpoint(setpoint.get()), this).onlyWhile(() -> !isAtSetpoint());
    }

    @Override
    public void periodic() {
        motor.setControl(motionMagic.withPosition(setpoint));
    }
}
