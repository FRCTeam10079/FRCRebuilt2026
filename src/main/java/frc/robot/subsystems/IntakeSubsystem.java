package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.generated.TunerConstants;

public class IntakeSubsystem extends SubsystemBase {
  // Initialize motors and encoder
  private final TalonFX intakeMotor =
      new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, TunerConstants.kCANBus);
  private final TalonFX pivotMotor =
      new TalonFX(IntakeConstants.PIVOT_MOTOR_ID, TunerConstants.kCANBus);
  private final CANcoder pivotEncoder =
      new CANcoder(IntakeConstants.PIVOT_ENCODER_ID, TunerConstants.kCANBus);
  // Current pivot setpoint
  private double pivotSetpoint = IntakeConstants.PIVOT_STOWED_POSITION;

  public IntakeSubsystem() {
    configurePivotMotor();
    configureIntakeMotor();
  }

  // Motor configurations (empty for now, can tune later)
  private void configurePivotMotor() {
    pivotMotor.setNeutralMode(NeutralModeValue.Brake); // Pivot should hold position
    // Add PID, current limits, soft limits,
  }

  private void configureIntakeMotor() {
    intakeMotor.setNeutralMode(NeutralModeValue.Coast); // Intake can spin freely
    // Add current limits, neutral mode,
  }

  // Pivot commands
  public void deployPivot() {
    pivotSetpoint = IntakeConstants.PIVOT_INTAKE_POSITION;
  }

  public void stowPivot() {
    pivotSetpoint = IntakeConstants.PIVOT_STOWED_POSITION;
  }

  // Check if pivot is deployed
  public boolean isDeployed() {
    // Check if pivot is actually at the intake position
    return Math.abs(getPivotPosition() - IntakeConstants.PIVOT_INTAKE_POSITION) < 0.05;
  }

  // Get pivot encoder position
  public double getPivotPosition() {
    return pivotEncoder.getPosition().getValueAsDouble();
  }

  // Intake motor commands
  public void intakeIn() {
    // Only spin intake if pivot is deployed
    if (isDeployed()) {
      intakeMotor.set(IntakeConstants.INTAKE_SPEED);
    }
  }

  public void stop() {
    intakeMotor.stopMotor();
  }
}
