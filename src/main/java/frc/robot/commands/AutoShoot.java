package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShoot extends Command {
  private final ShooterSubsystem m_shooter;
  private final LimelightSubsystem m_limelight;

  public AutoShoot(ShooterSubsystem shooter, LimelightSubsystem limelight) {
    m_shooter = shooter;
    m_limelight = limelight;
    addRequirements(
        shooter); // We don't require limelight because other commands might need to read from it
    // too
  }

  @Override
  public void execute() {
    // Check if we see a tag and if it is a hub
    if (!m_limelight.hasTarget() || !m_limelight.isValidHubTarget()) {
      // If we see a Trench tag (or nothing), stop the shooter so we don't waste power
      m_shooter.stop();
      return;
    }

    // 2. Get Distance (Now we know it's safe because it's definitely a Hub)
    double distance = m_limelight.getDistanceToGoal();

    // 3. Calculate RPM
    double targetRPM = m_shooter.getRpmForDistance(distance);

    // 4. Set Shooter
    m_shooter.setTargetRPM(targetRPM);
  }

  @Override
  public void end(boolean interrupted) {
    // When button released, stop the shooter
    m_shooter.stop();
  }
}
