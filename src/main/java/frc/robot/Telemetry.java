package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Telemetry {
  private final double m_maxSpeed;

  /* What to publish over networktables for telemetry */

  private final SwerveDriveTelemetry m_swerveDriveTelemetry;

  /* Robot pose for field positioning */
  private final DoubleArrayPublisher m_fieldPub;
  private final StringPublisher m_fieldTypePub;

  /* Mechanisms to represent the swerve module states */
  private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[4];
  /* A direction and length changing ligament for speed representation */
  private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[4];
  /* A direction changing and length constant ligament for module direction */
  private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[4];

  private final double[] m_poseArray = new double[3];
  private final double[] m_moduleStatesArray = new double[8];
  private final double[] m_moduleTargetsArray = new double[8];

  /**
   * Construct a telemetry object, with the specified max speed of the robot
   *
   * @param maxSpeed Maximum speed in meters per second
   */
  public Telemetry(double maxSpeed) {
    final var instance = NetworkTableInstance.getDefault();
    m_swerveDriveTelemetry = new SwerveDriveTelemetry(instance);

    final NetworkTable table = instance.getTable("Pose");
    m_fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    m_fieldTypePub = table.getStringTopic(".type").publish();

    for (int i = 0; i < 4; i++) {
      m_moduleMechanisms[i] = new Mechanism2d(1, 1);
      m_moduleSpeeds[i] = m_moduleMechanisms[i]
          .getRoot("RootSpeed", 0.5, 0.5)
          .append(new MechanismLigament2d("Speed", 0.5, 0));
      m_moduleDirections[i] = m_moduleMechanisms[i]
          .getRoot("RootDirection", 0.5, 0.5)
          .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite)));

      /* Set up the module state Mechanism2d telemetry */
      SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
    }

    m_maxSpeed = maxSpeed;
    SignalLogger.start();
  }

  /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
  public void telemeterize(SwerveDriveState state) {
    m_swerveDriveTelemetry.telemeterize(state);

    /* Also write to log file */
    m_poseArray[0] = state.Pose.getX();
    m_poseArray[1] = state.Pose.getY();
    m_poseArray[2] = state.Pose.getRotation().getDegrees();
    for (int i = 0; i < 4; i++) {
      m_moduleStatesArray[i * 2] = state.ModuleStates[i].angle.getRadians();
      m_moduleStatesArray[i * 2 + 1] = state.ModuleStates[i].speedMetersPerSecond;

      m_moduleTargetsArray[i * 2] = state.ModuleTargets[i].angle.getRadians();
      m_moduleTargetsArray[i * 2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
    }

    SignalLogger.writeDoubleArray("DriveState/Pose", m_poseArray);
    SignalLogger.writeDoubleArray("DriveState/ModuleStates", m_moduleStatesArray);
    SignalLogger.writeDoubleArray("DriveState/ModuleTargets", m_moduleTargetsArray);
    SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

    /* Telemeterize the pose to a Field2d */
    m_fieldTypePub.set("Field2d");
    m_fieldPub.set(m_poseArray);

    /* Telemeterize each module state to a Mechanism2d */
    for (int i = 0; i < 4; i++) {
      m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
      m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
      m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * m_maxSpeed));
    }
  }
}

class SwerveDriveTelemetry {
  private final StructPublisher<Pose2d> m_drivePose;
  private final StructPublisher<ChassisSpeeds> m_driveSpeeds;
  private final StructArrayPublisher<SwerveModuleState> m_driveModuleStates;
  private final StructArrayPublisher<SwerveModuleState> m_driveModuleTargets;
  private final StructArrayPublisher<SwerveModulePosition> m_driveModulePositions;
  private final DoublePublisher m_driveTimestamp;
  private final DoublePublisher m_driveOdometryFrequency;

  public SwerveDriveTelemetry(final NetworkTableInstance instance) {
    final NetworkTable driveStateTable = instance.getTable("DriveState");

    m_drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    m_driveSpeeds =
        driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    m_driveModuleStates = driveStateTable
        .getStructArrayTopic("ModuleStates", SwerveModuleState.struct)
        .publish();
    m_driveModuleTargets = driveStateTable
        .getStructArrayTopic("ModuleTargets", SwerveModuleState.struct)
        .publish();
    m_driveModulePositions = driveStateTable
        .getStructArrayTopic("ModulePositions", SwerveModulePosition.struct)
        .publish();
    m_driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    m_driveOdometryFrequency =
        driveStateTable.getDoubleTopic("OdometryFrequency").publish();
  }

  public void telemeterize(SwerveDriveState state) {
    m_drivePose.set(state.Pose);
    m_driveSpeeds.set(state.Speeds);
    m_driveModuleStates.set(state.ModuleStates);
    m_driveModuleTargets.set(state.ModuleTargets);
    m_driveModulePositions.set(state.ModulePositions);
    m_driveTimestamp.set(state.Timestamp);
    m_driveOdometryFrequency.set(1.0 / state.OdometryPeriod);
  }
}
