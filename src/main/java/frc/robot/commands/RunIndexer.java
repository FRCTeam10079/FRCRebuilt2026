package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class RunIndexer extends Command {
  private final IndexerSubsystem m_indexer;
  private final double m_speed;

  public RunIndexer(IndexerSubsystem indexer, double speed) {
    m_indexer = indexer;
    m_speed = speed;

    // Tell scheduler that we are using indexer
    addRequirements(indexer);
  }

  // Turn on when command starts
  @Override
  public void initialize() {
    m_indexer.setSpeed(m_speed);
  }

  @Override
  public void execute() {
    // Forces motor to keep spinning even if a CAN packet was dropped.
    m_indexer.setSpeed(m_speed);
  }

  // Turn off when command ends (button released)
  @Override
  public void end(boolean interrupted) {
    m_indexer.stop();
  }

  // Keep running until interrupted
  @Override
  public boolean isFinished() {
    return false;
  }
}
