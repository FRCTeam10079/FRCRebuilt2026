package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.generated.TunerConstants;

public class IndexerSubsystem extends SubsystemBase {
  // Use the generated CANBus instance instead of the deprecated String-based
  // constructor
  private final TalonFX m_feederMotor =
      new TalonFX(IndexerConstants.kFeederMotorID, TunerConstants.kCANBus);
  // // Follower commented out to see if it helps with issues
  // private final TalonFX m_spindexerMotor;

  public IndexerSubsystem() {
    // // Commented out to see if it helps with issues
    // m_spindexerMotor = new TalonFX(IndexerConstants.kSpindexerMotorID, kCANbus);

    var config = new TalonFXConfiguration();

    // Stator Current Limit
    config.CurrentLimits.StatorCurrentLimit = IndexerConstants.kCurrentLimit; // 40A

    // Supply limit
    // Prevents the motor from drawing too much from the battery and browning out
    // the robot.
    config.CurrentLimits.SupplyCurrentLimit = 40; // 30 Amps from battery

    // Brake Mode
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply config to the LEADER
    m_feederMotor.getConfigurator().apply(config);

    // Configure the FOLLOWER
    // We apply the same safety config to the follower too, just to be safe.
    // Commented out to see if it helps with issues
    // m_spindexerMotor.getConfigurator().apply(config);

    // Tell the follower to listen to the leader
    // Commented out to see if it helps with issues
    // m_spindexerMotor.setControl(new Follower(IndexerConstants.kFeederMotorID,
    // false));
  }

  public void setSpeed(double speed) {
    // Only command the leader, follow does it automatically
    m_feederMotor.set(speed);
  }

  public void stop() {
    m_feederMotor.stopMotor();
  }
}
