package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.generated.TunerConstants;

public class PivotSubsystem extends SubsystemBase {

  private final TalonFX pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID, TunerConstants.kCANBus);
  private double pivotSetpoint = IntakeConstants.PIVOT_STOWED_POSITION;

  public PivotSubsystem() {
    configurePivotMotor();
  }

  // Configure Pivot Motor
  private void configurePivotMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // PID constants
    config.Slot0 = config.Slot0.withGravityType(GravityTypeValue.Arm_Cosine)
        .withKA(IntakeConstants.KA)
        .withKV(IntakeConstants.KV)
        .withKD(IntakeConstants.KD)
        .withKG(IntakeConstants.KG)
        .withKS(IntakeConstants.KS)
        .withKI(IntakeConstants.KI)
        .withKP(IntakeConstants.KP);

    config.Feedback.SensorToMechanismRatio = 41;

    config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.PIVOT_INTAKE_POSITION;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.PIVOT_STOWED_POSITION;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    pivotMotor.getConfigurator().apply(config);
  }

  public void setPivotPosition(double position) {
    pivotSetpoint = position;
  }

  public void deployPivot() {
    setPivotPosition(IntakeConstants.PIVOT_INTAKE_POSITION);
  }

  public void stowPivot() {
    setPivotPosition(IntakeConstants.PIVOT_STOWED_POSITION);
  }

  public boolean isDeployed() {
    return Math.abs(getPivotPosition() - IntakeConstants.PIVOT_INTAKE_POSITION) < 0.05;
  }

  public double getPivotPosition() {
    return pivotMotor.getRotorPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    PositionVoltage request = new PositionVoltage(pivotSetpoint).withSlot(0);
    pivotMotor.setControl(request);
  }
}
