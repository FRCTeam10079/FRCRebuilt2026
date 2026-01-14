package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command {
    private final ClimbSubsystem climbSubsystem;
    private final Constants.ClimbHeight climbHeight;

    public ClimbCommand(ClimbSubsystem climbSubsystem, Constants.ClimbHeight climbHeight) {
        this.climbSubsystem = climbSubsystem;
        this.climbHeight = climbHeight;
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        climbSubsystem.setClimbSetpoint(climbHeight.getHeight());
    }

    @Override
    public boolean isFinished() {
        return climbSubsystem.isAtSetpoint();
    }
}
