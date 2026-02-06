package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.*;

public class PivotHomingSubsystem extends SubsystemBase {
    TalonFX pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID, TunerConstants.kCANBus);
    CircularBuffer<Measure<CurrentUnit>> currentRingBuffer = new CircularBuffer<>(IntakeConstants.HOMING_RING_BUFFER_SIZE);
    CircularBuffer<Measure<AngularVelocityUnit>> velocityRingBuffer = new CircularBuffer<>(IntakeConstants.HOMING_RING_BUFFER_SIZE);
    Supplier<Current> currentSupplier = pivotMotor.getSupplyCurrent().asSupplier();
    Supplier<AngularVelocity> velocitySupplier = pivotMotor.getVelocity().asSupplier();

    boolean isAtCurrentLimit = false;
    boolean isAtVelocityLimit = false;

    @Override
    public void periodic() {
        currentRingBuffer.addLast(currentSupplier.get());
        velocityRingBuffer.addLast(velocitySupplier.get());

        isAtCurrentLimit = getAverage(Amps.zero(), currentRingBuffer).gt(IntakeConstants.HOMING_CURRENT_LIMIT);
        isAtVelocityLimit = getAverage(RotationsPerSecond.zero(), velocityRingBuffer).gt(IntakeConstants.HOMING_VELOCITY_LIMIT);
    }

    public boolean isHome() {
        return isAtCurrentLimit && isAtVelocityLimit;
    }

    public void startHoming() {
        pivotMotor.setControl(new VoltageOut(IntakeConstants.HOMING_VOLTAGE));
    }

    public void stopHoming() {
        pivotMotor.stopMotor();
    }

    private static <UNIT extends Unit> Measure<UNIT> getAverage(Measure<UNIT> initialValue, CircularBuffer<Measure<UNIT>> buffer) {
        Measure<UNIT> accumulator = initialValue;
        for (int i = 0; i < buffer.size(); i++) {
            accumulator = accumulator.plus(buffer.get(i));
        }
        return accumulator;
    }
}
