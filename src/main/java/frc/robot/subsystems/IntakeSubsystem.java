package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.generated.TunerConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX m_intakeMotor =
      new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, TunerConstants.kCANBus);

  public IntakeSubsystem() {}

  public void intakeIn() {
    m_intakeMotor.set(IntakeConstants.INTAKE_SPEED);
  }

  public void stop() {
    m_intakeMotor.stopMotor();
  }
}
