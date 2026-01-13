package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
    private TalonFX shooterMotor = new TalonFX(24);

    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    
    private double currentSetpointRPM = 0;

    public ShooterSubsystem(){
        configureShooterMotor();
    }
    
    public void setFiring(){
        Commands.runOnce(
            //                     Shooter max RPM * 0.8
            () -> shooterMotor.set(ShooterConstants.SHOOTER_FIRING_SPEED) 
        );
        //Commands.runOnce(
        //    //                     Shooter max RPM * 0.0
        //    () -> otherShooterMotor.set(-ShooterConstants.SHOOTER_FIRING_SPEED) // 0 rpm
        //);
        currentSetpointRPM = ShooterConstants.SHOOTER_MAX_RPM * 0.8;
        
    }

    public void setSpinup(){
        Commands.runOnce(
            //                     Shooter max RPM * 0.6
            () -> shooterMotor.set(ShooterConstants.SHOOTER_SPINUP_SPEED)
        );
        //Commands.runOnce(
        //    //                     Shooter max RPM * 0.0
        //    () -> otherShooterMotor.set(-ShooterConstants.SHOOTER_SPINUP_SPEED) // 0 rpm
        //);
        currentSetpointRPM = ShooterConstants.SHOOTER_MAX_RPM * 0.6;
        
    }
    public void setIdle(){
        Commands.runOnce(
            //                     Shooter max RPM * 0.0
            () -> shooterMotor.set(ShooterConstants.SHOOTER_IDLE_SPEED) // 0 rpm
        );
        //Commands.runOnce(
        //    //                     Shooter max RPM * 0.0
        //    () -> otherShooterMotor.set(-ShooterConstants.SHOOTER_IDLE_SPEED) // 0 rpm
        //);
        currentSetpointRPM = 0;
    }
    // TODO: Find a way to read the motor's velocity
   // public boolean speedAtSetpoint(){
   //     return Math.abs(setpointRPM - currentRPM) < ShooterConstants.SHOOTER_RPM_TOLERANCE;
   // }
    
    public void getMotorRPM(){

    }
    // TODO: PID in config
    private void configureShooterMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        shooterMotor.getConfigurator().apply(config);
        
        // Set position update frequency for better feedback
        shooterMotor.getPosition().setUpdateFrequency(100);
        shooterMotor.getPosition().setUpdateFrequency(100);
        shooterMotor.optimizeBusUtilization();
    }

    @Override
    public void periodic() {

    }
    
}
