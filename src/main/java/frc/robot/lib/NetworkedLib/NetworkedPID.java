package frc.robot.lib.NetworkedLib;

import edu.wpi.first.math.controller.PIDController;

/**
 * Exactly like the original PIDController Except it allows for quick tuning via network modified
 * values
 */
public class NetworkedPID extends PIDController {
  private static int instanceCount;

  private String m_name;

  private NetworkedDouble m_networkedKp;
  private NetworkedDouble m_networkedKi;
  private NetworkedDouble m_networkedKd;

  /**
   * Allocates a PIDController with the given constants for kp, ki, and kd and a default period of
   * 0.02 seconds.
   *
   * @param kp The proportional coefficient.
   * @param ki The integral coefficient.
   * @param kd The derivative coefficient.
   * @throws IllegalArgumentException if kp &lt; 0
   * @throws IllegalArgumentException if ki &lt; 0
   * @throws IllegalArgumentException if kd &lt; 0
   */
  public NetworkedPID(double kp, double ki, double kd) {
    super(kp, ki, kd);
    instanceCount++;

    m_name = "NetworkedPIDController" + instanceCount;

    setupNT(kp, ki, kd);
  }

  /**
   * Allocates a PIDController with the given constants for kp, ki, and kd and a default period of
   * 0.02 seconds. Takes in a name for the NetworkTable
   *
   * @param kp The proportional coefficient.
   * @param ki The integral coefficient.
   * @param kd The derivative coefficient.
   * @param name The name to use in the NetworkTable
   * @throws IllegalArgumentException if kp &lt; 0
   * @throws IllegalArgumentException if ki &lt; 0
   * @throws IllegalArgumentException if kd &lt; 0
   */
  public NetworkedPID(double kp, double ki, double kd, String name) {
    super(kp, ki, kd);
    instanceCount++;

    m_name = name;

    setupNT(kp, ki, kd);
  }

  /**
   * Allocates a PIDController with the given constants for kp, ki, and kd. Takes in a name for the
   * NetworkTable
   *
   * @param kp The proportional coefficient.
   * @param ki The integral coefficient.
   * @param kd The derivative coefficient.
   * @param period The period between controller updates in seconds.
   * @param name The name to use in the NetworkTable
   * @throws IllegalArgumentException if kp &lt; 0
   * @throws IllegalArgumentException if ki &lt; 0
   * @throws IllegalArgumentException if kd &lt; 0
   * @throws IllegalArgumentException if period &lt;= 0
   */
  public NetworkedPID(double kp, double ki, double kd, double period, String name) {
    super(kp, ki, kd, period);
    instanceCount++;

    m_name = name;

    setupNT(kp, ki, kd);
  }

  private void setupNT(double kp, double ki, double kd) {
    m_networkedKp = new NetworkedDouble("/NetworkedLib/" + m_name + "/kp", kp);
    m_networkedKi = new NetworkedDouble("/NetworkedLib/" + m_name + "/ki", ki);
    m_networkedKd = new NetworkedDouble("/NetworkedLib/" + m_name + "/kd", kd);
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   * @param setpoint The new setpoint of the controller.
   * @return The next controller output.
   */
  public double calculate(double measurement, double setpoint) {
    updatePID();
    return super.calculate(measurement, setpoint);
  }

  /** Updates the current PID with new values */
  public void updatePID() {
    this.setPID(m_networkedKp.get(), m_networkedKi.get(), m_networkedKd.get());
  }
}
