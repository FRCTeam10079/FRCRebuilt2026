package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
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
  
  // Assumes pivot is stowed
  private double pivotSetpoint = IntakeConstants.PIVOT_STOWED_POSITION;

  public IntakeSubsystem() {


    configurePivotMotor();
    configureIntakeMotor();
  }

  // Motor configurations (empty for now, can tune later)
  private void configurePivotMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // pivot holds position
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // BE PREPARED TO CHANGE

    config.Slot0 = config.Slot0
    .withGravityType(GravityTypeValue.Arm_Cosine) // sets it to an ARM
    .withKA(IntakeConstants.KA)
    .withKV(IntakeConstants.KV)
    .withKD(IntakeConstants.KD)
    .withKG(IntakeConstants.KG)
    .withKS(IntakeConstants.KS)
    .withKI(IntakeConstants.KI)
    .withKD(IntakeConstants.KD)
    .withKP(IntakeConstants.KP);

    config.Feedback.SensorToMechanismRatio = 41; // this is motor to mechanism ratio, not sure if relevant but I put it
    
    // Current limits for pivot motor ; maybe add more limits for stuff like stator and other
    config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT; // Supply current limit
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // IMPORTANT: may need to swap the forward and reverse limits!
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.PIVOT_INTAKE_POSITION; // Slightly past intake position
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.PIVOT_STOWED_POSITION; // Slightly past stowed
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    pivotMotor.getConfigurator().apply(config);
  }

  private void configureIntakeMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // easy to spin when neutral
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // 
    
    // Current limits for intake motor ; may add or remove based on neccessary
    config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT; // same as pivot, might be changed
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    
    intakeMotor.getConfigurator().apply(config);
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
