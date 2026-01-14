package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;

import java.util.function.Supplier;

public class ClimbCommand extends Command {
    private final ClimbSubsystem climbSubsystem;
    private final Supplier<Constants.ClimbHeight> climbHeight;

    public ClimbCommand(ClimbSubsystem climbSubsystem, Supplier<Constants.ClimbHeight> climbHeight) {
        this.climbSubsystem = climbSubsystem;
        this.climbHeight = climbHeight;
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        climbSubsystem.climbToHeight(climbHeight.get().getHeight());
    }

    @Override
    public boolean isFinished() {
        return climbSubsystem.isAtHeight();
    }
}
