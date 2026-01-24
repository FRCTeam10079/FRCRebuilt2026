package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, "canivore");

    public IntakeSubsystem() {}

    public void intakeIn() {
        intakeMotor.set(IntakeConstants.INTAKE_SPEED);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }
}
