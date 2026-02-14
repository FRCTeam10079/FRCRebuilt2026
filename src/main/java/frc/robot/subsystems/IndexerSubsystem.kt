package frc.robot.subsystems

import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.lib.seconds
import frc.robot.Constants.IndexerConstants
import frc.robot.generated.TunerConstants

class IndexerSubsystem : SubsystemBase() {
    // Use the generated CANBus instance instead of the deprecated String-based constructor
    private val feederMotor = TalonFX(IndexerConstants.kFeederMotorID, TunerConstants.kCANBus)

    // private final TalonFX spindexerMotor; // Follower commented out to see if it helps with issues
    init {

        // spindexerMotor = new TalonFX(IndexerConstants.kSpindexerMotorID, kCANbus); // Commented out
        // to see if it helps with issues
        TalonFXConfiguration().apply {
            CurrentLimits.apply {
                StatorCurrentLimit = IndexerConstants.kCurrentLimit.toDouble() // 40A
                StatorCurrentLimitEnable = true

                // Supply limit
                // Prevents the motor from drawing too much from   the battery and browning out the robot.
                SupplyCurrentLimit = 40.0 // 30 Amps from battery
                SupplyCurrentLimitEnable = true
            }

            // Ramp rate for protection
            OpenLoopRamps.apply {
                withDutyCycleOpenLoopRampPeriod(0.0.seconds)
                DutyCycleOpenLoopRampPeriod = 0.0 // 0.25 seconds to full speed
                VoltageOpenLoopRampPeriod = 0.0
            }

            // Brake Mode
            MotorOutput.NeutralMode = NeutralModeValue.Brake
        }.let(feederMotor.configurator::apply)

        // Configure the FOLLOWER
        // We apply the same safety config to the follower too, just to be safe.
        // Commented out to see if it helps with issues
        // spindexerMotor.getConfigurator().apply(config);

        // Tell the follower to listen to the leader
        // Commented out to see if it helps with issues
        // spindexerMotor.setControl(new Follower(IndexerConstants.kFeederMotorID, false));
    }

    fun setSpeed(speed: Double) {
        // Only command the leader, follow does it automatically
        feederMotor.set(speed)
    }

    fun stop() {
        feederMotor.stopMotor()
    }
}
