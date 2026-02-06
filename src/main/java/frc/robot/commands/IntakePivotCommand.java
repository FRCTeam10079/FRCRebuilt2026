package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class IntakePivotCommand extends Command {

  private final PivotSubsystem pivot;

  public IntakePivotCommand(PivotSubsystem pivot) {
    this.pivot = pivot;
    addRequirements(pivot);
  }

  @Override
  public void initialize() {
    // Just set the pivot to move to intake position
    pivot.deployPivot();
  }

  @Override
  public void execute() {
    // Nothing else needed; pivot periodic() will handle moving to setpoint
  }

  @Override
  public void end(boolean interrupted) {
    // Optional: you could stow pivot if command ends, but usually we leave it
  }

  @Override
  public boolean isFinished() {
    // Keep running until canceled
    return false;
  }

  // Static helper commands if you want quick one-liners
  public static Command deployPivot(PivotSubsystem pivot) {
    return Commands.runOnce(() -> pivot.deployPivot(), pivot);
  }

  public static Command stowPivot(PivotSubsystem pivot) {
    return Commands.runOnce(() -> pivot.stowPivot(), pivot);
  }

  public static Command deployPivotWithIntake(PivotSubsystem pivot, IntakeSubsystem intake) {
    return Commands.parallel(
        // Pivot moves to intake position
        new IntakePivotCommand(pivot),

        // Intake wheels start running
        new IntakeCommand(intake));
  }
}
