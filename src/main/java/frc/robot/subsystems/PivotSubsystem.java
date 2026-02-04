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

public class PivotSubsystem extends SubsystemBase {
    // Intialize Pivot Motors
    private final TalonFX pivotMotor =
        new TalonFX(IntakeConstants.PIVOT_MOTOR_ID, TunerConstants.kCANBus);

    private final CANcoder pivotEncoder =
        new CANcoder(IntakeConstants.PIVOT_ENCODER_ID, TunerConstants.kCANBus);

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

        config.Feedback.SensorToMechanismRatio = 41;

        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        // Prevent Pivot from moving past intake position
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
            IntakeConstants.PIVOT_INTAKE_POSITION;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // Prevent Pivot from moving past stowed position
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
            IntakeConstants.PIVOT_STOWED_POSITION;
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
        // To check if pivot is deployed we get pivot position and then subtract it by its intake positon
        return Math.abs(getPivotPosition()
            - IntakeConstants.PIVOT_INTAKE_POSITION) < 0.05;
    }

    public double getPivotPosition() {
        return pivotEncoder.getPosition().getValueAsDouble();
    }
    // Commands

    public Command deployPivotCommand() {
        return Commands.runOnce(() -> deployPivot(), this);
    }

    public Command stowPivotCommand() {
        return Commands.runOnce(() -> stowPivot(), this);
    }
}
