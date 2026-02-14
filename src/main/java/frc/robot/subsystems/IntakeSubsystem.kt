package frc.robot.subsystems

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.IntakeConstants
import frc.robot.generated.TunerConstants

class IntakeSubsystem : SubsystemBase() {
    private val intakeMotor = TalonFX(IntakeConstants.INTAKE_MOTOR_ID, TunerConstants.kCANBus)

    fun intakeIn() = intakeMotor.set(IntakeConstants.INTAKE_SPEED)

    fun stop() = intakeMotor.stopMotor()
}
