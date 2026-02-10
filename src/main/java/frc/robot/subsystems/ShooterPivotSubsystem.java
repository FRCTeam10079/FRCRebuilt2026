package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.generated.TunerConstants;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class ShooterPivotSubsystem extends SubsystemBase {
    enum Mode {
        NORMAL,
        HOMING,
    }

    final TalonFX pivotMotor = new TalonFX(ShooterConstants.PIVOT_MOTOR_ID, TunerConstants.kCANBus);
    final CANcoder encoder = new CANcoder(ShooterConstants.PIVOT_ENCODER_ID);
    final Supplier<Angle> encoderAngle = encoder.getPosition().asSupplier();

    // Assumes pivot is stowed
    Angle setpoint = ShooterConstants.PIVOT_STOWED_POSITION;
    Angle setpointOffset = Rotations.zero();

    CircularBuffer<Measure<CurrentUnit>> currentRingBuffer = new CircularBuffer<>(ShooterConstants.PIVOT_HOMING_RING_BUFFER_SIZE);
    CircularBuffer<Measure<AngularVelocityUnit>> velocityRingBuffer = new CircularBuffer<>(ShooterConstants.PIVOT_HOMING_RING_BUFFER_SIZE);
    Supplier<Current> currentSupplier = pivotMotor.getSupplyCurrent().asSupplier();
    Supplier<AngularVelocity> velocitySupplier = pivotMotor.getVelocity().asSupplier();
    MotionMagicVoltage pivotRequest = new MotionMagicVoltage(setpoint).withSlot(0);

    boolean isAtCurrentLimit = false;
    boolean isAtVelocityLimit = false;

    private Mode mode = Mode.NORMAL;

    public ShooterPivotSubsystem() {
        configure(Mode.NORMAL);
    }

    private void configure(Mode mode) {
        if (mode == this.mode) return;
        this.mode = mode;
        switch (mode) {
            case HOMING -> configureHomingMode();
            case NORMAL -> configureNormalMode();
        }
    }

    private void configureHomingMode() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        // Resist motion when no power is applied to hold pivot position
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.CurrentLimits.withSupplyCurrentLimit(ShooterConstants.PIVOT_HOMING_SUPPLY_CURRENT_LIMIT);
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        pivotMotor.getConfigurator().apply(config);
    }

    private void configureNormalMode() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        // Resist motion when no power is applied to hold pivot position
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // PID Pivot Constants
        config.Slot0.withGravityType(GravityTypeValue.Arm_Cosine);
        config.Slot0.withKA(ShooterConstants.PIVOT_kA);
        config.Slot0.withKV(ShooterConstants.PIVOT_kV);
        config.Slot0.withKD(ShooterConstants.PIVOT_kD);
        config.Slot0.withKG(ShooterConstants.PIVOT_kG);
        config.Slot0.withKS(ShooterConstants.PIVOT_kS);
        config.Slot0.withKI(ShooterConstants.PIVOT_kI);
        config.Slot0.withKP(ShooterConstants.PIVOT_kP);

        config.MotionMagic.withMotionMagicCruiseVelocity(ShooterConstants.PIVOT_VELOCITY); // 5 (mechanism) rotations per second cruise
        config.MotionMagic.withMotionMagicAcceleration(ShooterConstants.PIVOT_ACCELERATION); // Take approximately 0.5 seconds to reach max vel
        config.MotionMagic.withMotionMagicJerk(ShooterConstants.PIVOT_JERK);

        config.Feedback.SensorToMechanismRatio = 41.0; // Motor-to-pivot ratio

        config.CurrentLimits.withSupplyCurrentLimit(ShooterConstants.SUPPLY_CURRENT_LIMIT);
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Prevent Pivot from moving past intake position
        config.SoftwareLimitSwitch.withForwardSoftLimitThreshold(ShooterConstants.PIVOT_MAX_POSITION);
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        // Prevent Pivot from moving past stowed position
        config.SoftwareLimitSwitch.withReverseSoftLimitThreshold(ShooterConstants.PIVOT_STOWED_POSITION);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        pivotMotor.getConfigurator().apply(config);
    }

    public void setSetpoint(Angle angle) {
        configure(Mode.NORMAL);
        setpoint = angle;
    }

    public boolean atSetpoint() {
        return encoderAngle.get().isNear(setpoint, ShooterConstants.SETPOINT_TOLERANCE);
    }

    public void periodic() {
        currentRingBuffer.addLast(currentSupplier.get());
        velocityRingBuffer.addLast(velocitySupplier.get());

        isAtCurrentLimit = getAverage(Amps.zero(), currentRingBuffer).gt(ShooterConstants.PIVOT_HOMING_CURRENT_LIMIT);
        isAtVelocityLimit = getAverage(RotationsPerSecond.zero(), velocityRingBuffer).gt(ShooterConstants.PIVOT_HOMING_VELOCITY_LIMIT);

        if (mode == Mode.NORMAL) {
            pivotMotor.setControl(pivotRequest.withPosition(setpoint.minus(setpointOffset)));
        }
    }

    public boolean isHome() {
        return isAtCurrentLimit && isAtVelocityLimit;
    }

    public void startHoming() {
        configure(Mode.HOMING);
        pivotMotor.setControl(new VoltageOut(ShooterConstants.PIVOT_HOMING_VOLTAGE));
    }

    public void stopHoming() {
        configure(Mode.HOMING);
        pivotMotor.stopMotor();
    }

    public Command homeCommand() {
        return new PivotHomeCommand().withTimeout(ShooterConstants.PIVOT_HOMING_TIMEOUT);
    }

    public Command pivotCommand(Angle angle) {
        return new PivotRotateCommand(() -> angle);
    }

    public Command pivotCommand(Supplier<Angle> angle) {
        return new PivotRotateCommand(angle);
    }

    private static <U extends Unit> Measure<U> getAverage(Measure<U> initialValue, CircularBuffer<Measure<U>> buffer) {
        Measure<U> accumulator = initialValue;
        for (int i = 0; i < buffer.size(); i++) {
            accumulator = accumulator.plus(buffer.get(i));
        }
        return accumulator;
    }

    private class PivotHomeCommand extends Command {
        {
            addRequirements(ShooterPivotSubsystem.this);
        }

        @Override
        public void initialize() {
            startHoming();
        }

        @Override
        public void end(boolean interrupted) {
            if (!interrupted) {
                setpointOffset = encoderAngle.get();
            }
            stopHoming();
        }

        @Override
        public boolean isFinished() {
            return isHome();
        }
    }

    private class PivotRotateCommand extends Command {
        {
            addRequirements(ShooterPivotSubsystem.this);
        }

        final Supplier<Angle> setpoint;

        private PivotRotateCommand(Supplier<Angle> setpoint) {
            this.setpoint = setpoint;
        }

        @Override
        public void execute() {
            setSetpoint(setpoint.get());
        }

        @Override
        public boolean isFinished() {
            return atSetpoint();
        }
    }
}
