package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.generated.TunerConstants;

public class PivotSubsystem extends SubsystemBase {
  // Initialize Pivot Motor
  // Encoder is also intialized since encoder is built into the motor

  private final TalonFX pivotMotor =
      new TalonFX(IntakeConstants.PIVOT_MOTOR_ID, TunerConstants.kCANBus);

  // Assumes pivot is stowed
  private double pivotSetpoint = IntakeConstants.PIVOT_STOWED_POSITION;

  public PivotSubsystem() {
    configurePivotMotor();
  }

  private void configurePivotMotor() {
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
    pivotSetpoint = IntakeConstants.PIVOT_INTAKE_POSITION;
  }

  public void stowPivot() {
    pivotSetpoint = IntakeConstants.PIVOT_STOWED_POSITION;
  }

  public boolean isDeployed() {
    // Use motor encoder position instead of CANcoder
    return Math.abs(getPivotPosition() - IntakeConstants.PIVOT_INTAKE_POSITION) < 0.05;
  }

  // Read built-in motor encoder
  public double getPivotPosition() {
    // Use the TalonFX built-in motor encoder (rotor) to get position of the motor
    return pivotMotor.getRotorPosition().getValueAsDouble();
  }

  // Commands
  public Command deployPivotCommand() {
    return Commands.runOnce(() -> deployPivot(), this);
  }

  public Command stowPivotCommand() {
    return Commands.runOnce(() -> stowPivot(), this);
  }
}
