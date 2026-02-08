package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.generated.TunerConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor =
      new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, TunerConstants.kCANBus);

  public IntakeSubsystem() {
    configureIntakeMotor();
  }

  private void configureIntakeMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // wheels spin slowly to stop
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    intakeMotor.getConfigurator().apply(config);
  }

  public void intakeIn() {
    intakeMotor.set(IntakeConstants.INTAKE_SPEED);
  }

  public void stop() {
    intakeMotor.stopMotor();
  }
}
