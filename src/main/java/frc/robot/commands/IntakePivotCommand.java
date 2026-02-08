package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class IntakePivotCommand extends Command {

  private final IntakeSubsystem intake;
  private final PivotSubsystem pivot;

  public IntakePivotCommand(IntakeSubsystem intake, PivotSubsystem pivot) {
    this.intake = intake;
    this.pivot = pivot;
    addRequirements(intake, pivot);
  }

  @Override
  public void initialize() {
    // Start intake wheels
    intake.intakeIn();

    // Deploy pivot
    pivot.deployPivot();
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
    // Stop intake when command ends
    intake.stop();
    // Stow the Pivot
    pivot.stowPivot();
  }

  @Override
  public boolean isFinished() {

    return false;
  }
}
