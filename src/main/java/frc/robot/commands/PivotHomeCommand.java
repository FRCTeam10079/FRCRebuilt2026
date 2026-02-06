package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotHomingSubsystem;

public class PivotHomeCommand extends Command {
    final PivotHomingSubsystem pivotHomingSubsystem;

    /**
     * TODO: Add a PivotSubsystem requirement so the two motors don't run at the same time
     * @param pivotHomingSubsystem
     */
    public PivotHomeCommand(PivotHomingSubsystem pivotHomingSubsystem) {
        addRequirements(pivotHomingSubsystem);
        this.pivotHomingSubsystem = pivotHomingSubsystem;
    }

    @Override
    public void initialize() {
        pivotHomingSubsystem.startHoming();
    }

    @Override
    public void end(boolean interrupted) {
        pivotHomingSubsystem.stopHoming();
    }

    @Override
    public boolean isFinished() {
        return pivotHomingSubsystem.isHome();
    }
}
