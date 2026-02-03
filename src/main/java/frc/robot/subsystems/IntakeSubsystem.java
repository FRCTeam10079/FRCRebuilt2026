package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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

    private void configurePivotMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;// Defines ClockWise Positive as the positive side for rotation

        config.Slot0 = config.Slot0
            .withGravityType(GravityTypeValue.Arm_Cosine)
            //PID constants
            .withKA(IntakeConstants.KA)
            .withKV(IntakeConstants.KV)
            .withKD(IntakeConstants.KD)
            .withKG(IntakeConstants.KG)
            .withKS(IntakeConstants.KS)
            .withKI(IntakeConstants.KI)
            .withKP(IntakeConstants.KP);

        config.Feedback.SensorToMechanismRatio = 41; // Tells Motor how many motor rotations = 1 pivot rotation

        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT; 
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.PIVOT_INTAKE_POSITION;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.PIVOT_STOWED_POSITION;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        pivotMotor.getConfigurator().apply(config);
    }

    private void configureIntakeMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        intakeMotor.getConfigurator().apply(config);
    }

    // Pivot methods
    public void deployPivot() {
        pivotSetpoint = IntakeConstants.PIVOT_INTAKE_POSITION;
    }

    public void stowPivot() {
        pivotSetpoint = IntakeConstants.PIVOT_STOWED_POSITION;
    }

    // Check if pivot is deployed
    public boolean isDeployed() {
        return Math.abs(getPivotPosition() - IntakeConstants.PIVOT_INTAKE_POSITION) < 0.05;
    }

    // Get pivot encoder position
    public double getPivotPosition() {
        return pivotEncoder.getPosition().getValueAsDouble();
    }

    // Intake motor commands
    public void intakeIn() {
        if (isDeployed()) {
            intakeMotor.set(IntakeConstants.INTAKE_SPEED);
        }
    }

    public void stop() {
        intakeMotor.stopMotor();
    }

    // Commands
    public Command deployPivotCommand() {
        return Commands.runOnce(() -> deployPivot(), this);
    }

    public Command stowPivotCommand() {
        return Commands.runOnce(() -> stowPivot(), this);
    }
}
