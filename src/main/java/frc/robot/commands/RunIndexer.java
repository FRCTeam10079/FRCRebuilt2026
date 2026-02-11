package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class RunIndexer extends Command {

private final IndexerSubsystem m_indexer;
private final double m_feederRPM;
private final double m_spindexerRPM;

/**
* Runs the indexer with specific speeds for both motors.
*
* @param indexer The subsystem
* @param feederRPM Target RPM for the fast top roller
* @param spindexerRPM Target RPM for the floor/star wheel
*/
public RunIndexer(IndexerSubsystem indexer, double feederRPM, double spindexerRPM) {
m_indexer = indexer;
m_feederRPM = feederRPM;
m_spindexerRPM = spindexerRPM;

// Lock the subsystem so no one else uses it
addRequirements(indexer);
}

// Called when the command is initially scheduled.
@Override
public void initialize() {
m_indexer.setSpeeds(m_feederRPM, m_spindexerRPM);
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
// We call this continuously to ensure the motor controller keeps getting the signal
m_indexer.setSpeeds(m_feederRPM, m_spindexerRPM);
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
m_indexer.stop();
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
return false; // Runs until you release the button
}
}

