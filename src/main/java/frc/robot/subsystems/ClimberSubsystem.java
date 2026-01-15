package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
// This is testing code for the climber subsystem
public class ClimberSubsystem extends SubsystemBase {

    // The motor
    private final TalonFX climberMotor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_ID);

public ClimberSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();


        
        // 1. Protects battery/breaker
        config.CurrentLimits.SupplyCurrentLimit = ClimberConstants.CURRENT_LIMIT; // 60 Amps
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // 2. Prevents motor crunching
        // Limits torque applied by the motor
        // If the arm hits the bottom, the force spikes. So this will stop it that pushing higher than 80 amps of force
        config.CurrentLimits.StatorCurrentLimit = 80; 
        config.CurrentLimits.StatorCurrentLimitEnable = true;

       

        climberMotor.getConfigurator().apply(config);

        // Brake Mode 
        climberMotor.setNeutralMode(NeutralModeValue.Brake);
        
        // Reset the counter to 0 when the robot turns on. 
        // Always start the robot with the climber fully down(retracted)
        climberMotor.setPosition(0);
    }


    public void move(double speed) {
        climberMotor.set(speed);
    }

// Stops motor from being at current position 
    public void stop() {
        climberMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // For later
    }
}