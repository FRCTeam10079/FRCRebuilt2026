// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.generated.TunerConstants;

/**
 * This subsystem contains: - Debounced "isReady()" check to ensure flywheel stability before
 * feeding - Single motor configuration (dual motor support commented out for future use) -
 * Auto-Ranging Lookup Table implementation
 */
public class ShooterSubsystem extends SubsystemBase {
  // *** UPDATED: Using TunerConstants.kCANBus ***
  private final TalonFX m_masterMotor =
      new TalonFX(ShooterConstants.MASTER_MOTOR_ID, TunerConstants.kCANBus);
  // DUAL MOTOR: Uncomment for 2-motor setup
  // private final TalonFX m_slaveMotor;

  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
  private final NeutralOut m_neutralRequest = new NeutralOut();
  // DUAL MOTOR: Uncomment for 2-motor setup
  // private final Follower m_followerRequest;

  private double m_targetRPM;
  private boolean m_isEnabled;

  // Stability tracking
  private int m_stabilityCounter;
  private static final int STABILITY_CYCLES_REQUIRED = 5;

  // Lookup Table for Auto-Ranging
  private final InterpolatingDoubleTreeMap m_distanceToRPMTable = new InterpolatingDoubleTreeMap();

  /** Creates a new ShooterSubsystem */
  public ShooterSubsystem() {
    // DUAL MOTOR: Uncomment for 2-motor setup
    // m_slaveMotor = new TalonFX(ShooterConstants.SLAVE_MOTOR_ID,
    // TunerConstants.kCANBus);

    configureMotors();

    // DUAL MOTOR: Uncomment for 2-motor setup
    // m_followerRequest = new Follower(ShooterConstants.MASTER_MOTOR_ID, true);

    // Load the Lookup Table from Constants
    for (double[] pair : ShooterConstants.DISTANCE_TO_RPM_MAP) {
      m_distanceToRPMTable.put(pair[0], pair[1]);
    }
  }

  /** Configure both shooter motors with appropriate gains and limits */
  private void configureMotors() {
    TalonFXConfiguration masterConfig = new TalonFXConfiguration();

    masterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    masterConfig.Slot0 = new Slot0Configs()
        .withKS(ShooterConstants.SHOOTER_KS)
        .withKV(ShooterConstants.SHOOTER_KV)
        .withKP(ShooterConstants.SHOOTER_KP);

    m_masterMotor.getConfigurator().apply(masterConfig);
  }

  @Override
  public void periodic() {
    // Read current velocity from motor
    double currentRPM = getCurrentRPM();

    // Update stability counter (debouncing logic)
    if (m_isEnabled && m_targetRPM != 0) {
      double error = Math.abs(m_targetRPM - currentRPM);
      if (error <= ShooterConstants.SHOOTER_RPM_TOLERANCE) {
        m_stabilityCounter = Math.min(m_stabilityCounter + 1, STABILITY_CYCLES_REQUIRED);
      } else {
        m_stabilityCounter = 0;
      }
      // Apply control to motors
      double targetRps = m_targetRPM / 60.0;
      m_masterMotor.setControl(m_velocityRequest.withVelocity(targetRps));
    } else {
      m_stabilityCounter = 0;
      // Apply control to motors
      m_masterMotor.setControl(m_neutralRequest);
    }

    // Telemetry
    SmartDashboard.putNumber("Shooter/TargetRPM", m_targetRPM);
    SmartDashboard.putNumber("Shooter/CurrentRPM", currentRPM);
    SmartDashboard.putBoolean("Shooter/IsReady", isReady());
  }

  /**
   * Calculates the ideal RPM for a specific distance using the Lookup Table.
   *
   * @param distanceMeters The distance from the target (calculated by Limelight).
   * @return The calculated RPM.
   */
  public double getRPMForDistance(double distanceMeters) {
    return m_distanceToRPMTable.get(distanceMeters);
  }

  /**
   * Set the target RPM for the shooter flywheels
   *
   * @param rpm Target velocity in rotations per minute
   */
  public void setTargetRPM(double rpm) {
    m_targetRPM = Math.max(0, Math.min(rpm, ShooterConstants.SHOOTER_MAX_RPM));
    m_isEnabled = rpm > 0;

    if (Math.abs(rpm - m_targetRPM) > ShooterConstants.SHOOTER_RPM_TOLERANCE) {
      m_stabilityCounter = 0;
    }
  }

  /** Enable the shooter at the pre-configured spin-up RPM */
  public void spinUp() {
    setTargetRPM(ShooterConstants.SHOOTER_SPINUP_RPM);
  }

  /** Stop the shooter (coast to stop) */
  public void stop() {
    m_targetRPM = 0;
    m_isEnabled = false;
    m_stabilityCounter = 0;
  }

  public boolean isReady() {
    return m_isEnabled && m_targetRPM != 0 && m_stabilityCounter == STABILITY_CYCLES_REQUIRED;
  }

  public boolean isAtSetpoint() {
    return m_isEnabled
        && m_targetRPM != 0
        && Math.abs(m_targetRPM - getCurrentRPM()) <= ShooterConstants.SHOOTER_RPM_TOLERANCE;
  }

  public double getCurrentRPM() {
    return m_masterMotor.getVelocity().getValueAsDouble() * 60.0;
  }

  // Command Factories
  public Command spinUpCommand() {
    return runOnce(this::spinUp).withName("Shooter Spin Up");
  }

  public Command stopCommand() {
    return runOnce(this::stop).withName("Shooter Stop");
  }
}
