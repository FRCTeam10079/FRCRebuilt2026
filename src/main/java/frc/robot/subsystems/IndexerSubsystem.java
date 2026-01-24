package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {

    private final TalonFX indexerMotor;

    public IndexerSubsystem() {
        indexerMotor = new TalonFX(IndexerConstants.kIndexerMotorID);

        
        TalonFXConfiguration config = new TalonFXConfiguration();

        //Current limit
        config.CurrentLimits.StatorCurrentLimit = IndexerConstants.kCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Brake mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Apply config
        indexerMotor.getConfigurator().apply(config);
    }

    // Run the motor at a specific percentage (-1.0 to 1.0)
    public void setSpeed(double speed) {
        indexerMotor.set(speed);
    }

    // Stop the motor
    public void stop() {
        indexerMotor.stopMotor();
    }
}