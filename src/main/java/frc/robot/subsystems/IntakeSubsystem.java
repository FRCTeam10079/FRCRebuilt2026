package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.generated.TunerConstants;

public class IntakeSubsystem extends SubsystemBase {
  // Initialize motors and encoder
  private final TalonFX intakeMotor =
      new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, TunerConstants.kCANBus);

  private final PivotSubsystem pivot;

  public IntakeSubsystem(PivotSubsystem pivot) {
    this.pivot = pivot;
    configureIntakeMotor();
  }

  private void configureIntakeMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    // Intake Rollers spin even if no power is applied
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    //
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // Sets Current Limits
    config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT;
    // Enable Current Limits
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    intakeMotor.getConfigurator().apply(config);
  }

  public void intakeIn() {
    // Spinup is handled by the DeployPivotWithSpinup.java command
    intakeMotor.set(IntakeConstants.INTAKE_SPEED);
  }

  public void stop() {
    intakeMotor.stopMotor();
  }
}