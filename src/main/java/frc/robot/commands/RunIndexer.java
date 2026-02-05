package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class RunIndexer extends Command {

  private final IndexerSubsystem indexer;
  private final double speed;

  public RunIndexer(IndexerSubsystem indexer, double speed) {
    this.indexer = indexer;
    this.speed = speed;

    // Tell scheduler that we are using indexer
    addRequirements(indexer);
  }

  // Turn on when command starts
  @Override
  public void initialize() {
    indexer.setSpeed(speed);
  }

  @Override
  public void execute() {
    // Forces motor to keep spinning even if a CAN packet was dropped.
    indexer.setSpeed(speed);
  }

  // Turn off when command ends (button released)
  @Override
  public void end(boolean interrupted) {
    indexer.stop();
  }

  // Keep running until interrupted
  @Override
  public boolean isFinished() {
    return false;
  }
}
