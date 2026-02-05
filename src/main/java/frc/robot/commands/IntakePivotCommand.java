package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePivotCommand {

    // Sequence: deploy pivot and spin up intake
    public static SequentialCommandGroup deployWithSpinup(PivotSubsystem pivot, IntakeSubsystem intake) {
        return new SequentialCommandGroup(
                // Step 1: move pivot down
                new InstantCommand(() -> pivot.deployPivot(), pivot),
                // Step 2: wait a little so intake spins up before pivot reaches bottom
                new WaitCommand(0.3), // spinup delay for intake
                // Step 3: start intake
                new InstantCommand(() -> intake.intakeIn(), intake),
                // Step 4: wait until pivot fully deployed
                new WaitUntilCommand(pivot::isDeployed),
                // Step 5: stop intake
                new InstantCommand(() -> intake.stop(), intake));
    }

    public static Command deployPivot(PivotSubsystem pivot) {
        return Commands.runOnce(() -> pivot.deployPivot(), pivot);
    }

    public static Command stowPivot(PivotSubsystem pivot) {
        return Commands.runOnce(() -> pivot.stowPivot(), pivot);
    }

}
