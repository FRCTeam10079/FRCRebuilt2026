package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.generated.TunerConstants;

public class IndexerSubsystem extends SubsystemBase {

  private final TalonFX feederMotor;
  // private final TalonFX spindexerMotor; // Follower commented out to see if it helps with issues

  public IndexerSubsystem() {
    // Use the generated CANBus instance instead of the deprecated String-based constructor
    feederMotor = new TalonFX(IndexerConstants.kFeederMotorID, TunerConstants.kCANBus);
    // spindexerMotor = new TalonFX(IndexerConstants.kSpindexerMotorID, kCANbus); // Commented out
    // to see if it helps with issues

    TalonFXConfiguration config = new TalonFXConfiguration();

    // Stator Current Limit
    config.CurrentLimits.StatorCurrentLimit = IndexerConstants.kCurrentLimit; // 40A
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // Supply limit
    // Prevents the motor from drawing too much from   the battery and browning out the robot.
    config.CurrentLimits.SupplyCurrentLimit = 40; // 30 Amps from battery
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Ramp rate for protection
    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0; // 0.25 seconds to full speed
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;

    // Brake Mode
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply config to the LEADER
    feederMotor.getConfigurator().apply(config);

    // Configure the FOLLOWER
    // We apply the same safety config to the follower too, just to be safe.
    // Commented out to see if it helps with issues
    // spindexerMotor.getConfigurator().apply(config);

    // Tell the follower to listen to the leader
    // Commented out to see if it helps with issues
    // spindexerMotor.setControl(new Follower(IndexerConstants.kFeederMotorID, false));
  }

  public void setSpeed(double speed) {
    // Only command the leader, follow does it automatically
    feederMotor.set(speed);
  }

  public void stop() {
    feederMotor.stopMotor();
  }
}
