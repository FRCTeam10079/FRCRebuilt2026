package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeWheelSubsystem;

public class IntakeCommand extends Command {
    private final IntakeWheelSubsystem intake;
    private final double speed;

    public IntakeCommand(IntakeWheelSubsystem intake, double speed) {
        this.intake = intake;
        this.speed = speed;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakePower(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
