package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    
    // hardware
    private final TalonFX shooterMotor = new TalonFX(ShooterConstants.SHOOTER_MOTOR_ID); // note that shooter motor id is a placeholder

    // controls
    // uses voltage to keep speed steady even if battery is low
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

    // target speed
    private double currentSetpointRPM = 0;

    public ShooterSubsystem() {
        configureShooterMotor();
    }

    private void configureShooterMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // math to keep speed steady
        Slot0Configs slot0 = config.Slot0;
        slot0.kP = ShooterConstants.kP;
        slot0.kI = ShooterConstants.kI;
        slot0.kD = ShooterConstants.kD;
        slot0.kV = ShooterConstants.kV;

        // safety limits
        config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // apply settings
        shooterMotor.getConfigurator().apply(config);
        
        // coast so it doesn't jerk when stopping
        shooterMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    // main function to set speed
    public void setTargetRPM(double rpm) {
        currentSetpointRPM = rpm;
        
        // convert RPM to RPS for the motor
        double rps = currentSetpointRPM / 60.0;
        
        shooterMotor.setControl(velocityRequest.withVelocity(rps));
    }

    // turn off motor
    public void stop() {
        currentSetpointRPM = 0;
        shooterMotor.stopMotor();
    }

    // checks if we are fast enough to shoot
    public boolean isReadyToFire() {
        double currentRPM = getMotorRPM();
        double error = Math.abs(currentSetpointRPM - currentRPM);
        
        // true if we are trying to spin AND close to target
        return currentSetpointRPM > 0 && error < ShooterConstants.SHOOTER_RPM_TOLERANCE;
    }

    // get real speed
    public double getMotorRPM() {
        // convert RPS back to RPM
        return shooterMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    // preset for high goal
    public void setFiring() {
        setTargetRPM(ShooterConstants.SHOOTER_FIRING_RPM);
    }

    // preset to keep warm
    public void setIdle() {
        setTargetRPM(ShooterConstants.SHOOTER_IDLE_RPM);
    }

    @Override
    public void periodic() {
        // data for dashboard
        SmartDashboard.putNumber("Shooter/Current RPM", getMotorRPM());
        SmartDashboard.putNumber("Shooter/Target RPM", currentSetpointRPM);
        SmartDashboard.putBoolean("Shooter/Ready", isReadyToFire());
        
        // useful for tuning
        SmartDashboard.putNumber("Shooter/Applied Volts", shooterMotor.getMotorVoltage().getValueAsDouble());
    }
}
