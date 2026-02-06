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
import frc.robot.Constants.IntakeConstants;
import frc.robot.generated.TunerConstants;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class PivotSubsystem extends SubsystemBase {
    enum Mode {
        NORMAL,
        HOMING,
    }

    private final TalonFX pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID, TunerConstants.kCANBus);
    private final CANcoder encoder = new CANcoder(IntakeConstants.PIVOT_ENCODER_ID);
    private final Supplier<Angle> encoderAngle = encoder.getPosition().asSupplier();

    // Assumes pivot is stowed
    private double pivotSetpoint = IntakeConstants.PIVOT_STOWED_POSITION;
    double setpointOffset = 0.0;

    CircularBuffer<Measure<CurrentUnit>> currentRingBuffer = new CircularBuffer<>(IntakeConstants.HOMING_RING_BUFFER_SIZE);
    CircularBuffer<Measure<AngularVelocityUnit>> velocityRingBuffer = new CircularBuffer<>(IntakeConstants.HOMING_RING_BUFFER_SIZE);
    Supplier<Current> currentSupplier = pivotMotor.getSupplyCurrent().asSupplier();
    Supplier<AngularVelocity> velocitySupplier = pivotMotor.getVelocity().asSupplier();
    MotionMagicVoltage pivotRequest = new MotionMagicVoltage(pivotSetpoint).withSlot(0);
    boolean isAtCurrentLimit = false;
    boolean isAtVelocityLimit = false;

    private Mode mode = Mode.NORMAL;

    public PivotSubsystem() {
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

        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.HOMING_SUPPLY_CURRENT_LIMIT.in(Amps);
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        pivotMotor.getConfigurator().apply(config);
    }

    private void configureNormalMode() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        // Resist motion when no power is applied to hold pivot position
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Slot0 = config.Slot0
                // PID Pivot Constants
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKA(IntakeConstants.KA)
                .withKV(IntakeConstants.KV)
                .withKD(IntakeConstants.KD)
                .withKG(IntakeConstants.KG)
                .withKS(IntakeConstants.KS)
                .withKI(IntakeConstants.KI)
                .withKP(IntakeConstants.KP);

        config.MotionMagic
                .withMotionMagicCruiseVelocity(IntakeConstants.PIVOT_VELOCITY) // 5 (mechanism) rotations per second cruise
                .withMotionMagicAcceleration(IntakeConstants.PIVOT_ACCELERATION) // Take approximately 0.5 seconds to reach max vel
                .withMotionMagicJerk(IntakeConstants.PIVOT_JERK);

        config.Feedback.SensorToMechanismRatio = 41; // Motor-to-pivot ratio

        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Prevent Pivot from moving past intake position
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.PIVOT_INTAKE_POSITION;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        // Prevent Pivot from moving past stowed position
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.PIVOT_STOWED_POSITION;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        pivotMotor.getConfigurator().apply(config);
    }

    public void deployPivot() {
        configure(Mode.NORMAL);
        pivotSetpoint = IntakeConstants.PIVOT_INTAKE_POSITION;
    }

    public void stowPivot() {
        configure(Mode.NORMAL);
        pivotSetpoint = IntakeConstants.PIVOT_STOWED_POSITION;
    }

    public boolean isDeployed() {
        // Use motor encoder position instead of CANcoder
        return Math.abs(getPivotPosition() - IntakeConstants.PIVOT_INTAKE_POSITION)
                < 0.05; // may need to change
    }

    // Read built-in motor encoder
    public double getPivotPosition() {
        // Use the TalonFX built-in motor encoder (rotor) to get position of the motor
        return pivotMotor.getRotorPosition().getValueAsDouble();
    }

    public void periodic() {
        currentRingBuffer.addLast(currentSupplier.get());
        velocityRingBuffer.addLast(velocitySupplier.get());

        isAtCurrentLimit = getAverage(Amps.zero(), currentRingBuffer).gt(IntakeConstants.HOMING_CURRENT_LIMIT);
        isAtVelocityLimit = getAverage(RotationsPerSecond.zero(), velocityRingBuffer).gt(IntakeConstants.HOMING_VELOCITY_LIMIT);

        if (mode == Mode.NORMAL) {
            pivotMotor.setControl(pivotRequest.withPosition(pivotSetpoint - setpointOffset));
        }
    }
    // Commands

    public boolean isHome() {
        return isAtCurrentLimit && isAtVelocityLimit;
    }

    public void startHoming() {
        configure(Mode.HOMING);
        pivotMotor.setControl(new VoltageOut(IntakeConstants.HOMING_VOLTAGE));
    }

    public void stopHoming() {
        configure(Mode.HOMING);
        pivotMotor.stopMotor();
    }

    public Command homeCommand() {
        return new PivotHomeCommand().withTimeout(IntakeConstants.HOMING_TIMEOUT);
    }

    private static <UNIT extends Unit> Measure<UNIT> getAverage(Measure<UNIT> initialValue, CircularBuffer<Measure<UNIT>> buffer) {
        Measure<UNIT> accumulator = initialValue;
        for (int i = 0; i < buffer.size(); i++) {
            accumulator = accumulator.plus(buffer.get(i));
        }
        return accumulator;
    }

    private class PivotHomeCommand extends Command {
        {
            addRequirements(PivotSubsystem.this);
        }

        @Override
        public void initialize() {
            startHoming();
        }

        @Override
        public void end(boolean interrupted) {
            if (!interrupted) {
                setpointOffset = encoderAngle.get().in(Rotations);
            }
            stopHoming();
        }

        @Override
        public boolean isFinished() {
            return isHome();
        }
    }
}
