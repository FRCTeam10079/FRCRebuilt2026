
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;



public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID); // replace with real CAN ID

  public IntakeSubsystem() {
  }

  public void intakeIn() {
    intakeMotor.set(0.6);
    // example speed not actual motor command
  }



  public void stop() {
    intakeMotor.stopMotor();
  }
}
