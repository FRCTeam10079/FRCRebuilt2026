package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.IntakeCommand;

public class IntakeWheelSubsystem extends SubsystemBase {

    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);

    public void setIntakePower(double speed) {
        intakeMotor.set(speed);
    }

    public void stopIntake() {
        intakeMotor.stopMotor();
    }

    public void stop() {
        intakeMotor.stopMotor();
    }

    /**
     * Run the intake at the given power (never ending).
     *
     * @param power The power to run the intake at.
     * @return The command
     */
    public IntakeCommand runIntakeCommand(double power) {
        return new IntakeCommand(this, power);
    }

    /**
     * Run the intake at the default intake power (never ending).
     *
     * @return The command
     */
    public IntakeCommand runIntakeCommand() {
        return runIntakeCommand(IntakeConstants.INTAKE_SPEED);
    }

    /**
     * Run the intake at the default outtake power (never ending).
     *
     * @return The command
     */
    public IntakeCommand runOuttakeCommand() {
        return runIntakeCommand(IntakeConstants.OUTTAKE_SPEED);
    }

    /**
     * Starts the intake at the given power (immediately ends).
     *
     * @param power The power to run the intake at.
     * @return The command
     */
    public Command startIntakeCommand(double power) {
        return Commands.runOnce(() -> setIntakePower(power), this);
    }

    /**
     * Starts the intake at the default intake power (immediately ends).
     *
     * @return The command
     */
    public Command startIntakeCommand() {
        return startIntakeCommand(IntakeConstants.INTAKE_SPEED);
    }

    /**
     * Starts the outtake at the default outtake power (immediately ends).
     *
     * @return The command
     */
    public Command startOuttakeCommand() {
        return startIntakeCommand(IntakeConstants.OUTTAKE_SPEED);
    }

    /**
     * Stops the intake.
     *
     * @return The command
     */
    public Command stopCommand() {
        return Commands.runOnce(this::stop, this);
    }
}
