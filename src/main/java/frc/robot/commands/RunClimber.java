package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
// This is testing code for the climber command
// Note that this is if we have a climb where there is a hook that pulls the robot up
public class RunClimber extends Command {
    
    private final ClimberSubsystem climber;
    private final DoubleSupplier speedInput; // Joystick axis input


    public RunClimber(ClimberSubsystem climber, DoubleSupplier speedInput) {
        this.climber = climber;
        this.speedInput = speedInput;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        // Get the value from the joystick (-1.0 to 1.0)
        double speed = speedInput.getAsDouble();

        
        // If the driver stick is barely moving, ignore it
        if (Math.abs(speed) < 0.1) {
            speed = 0;
        }

        // Send to subsystem
        climber.move(speed);
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs as long as we tell it to 
    }
}