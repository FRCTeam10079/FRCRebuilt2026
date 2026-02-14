package frc.robot.lib.NetworkedLib;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

/*
 * TalonFX motor but slot0 is dynamic
 *
 * WARNING: THIS ONLY WORKS WITH SLOT 0 DO NOT ATTEMPT TO USE OTHER SLOTS
 * (this is not designed to be production code, this is code meant for fast prototyping)
 */
public class NetworkedTalonFX extends TalonFX {
  private static int instanceCount;

  private String m_name;

  // networked prefix to just to avoid ambigiouity between actual values and their
  // networked
  // partners
  private NetworkedDouble m_networkedKA; // acceleration gain
  private NetworkedDouble m_networkedkD; // derivative gain
  private NetworkedDouble m_networkedkG; // gravity gain
  private NetworkedDouble m_networkedkI; // integral gain
  private NetworkedDouble m_networkedkP; // proprotional gain
  private NetworkedDouble m_networkedKS; // static constant (static friction)
  private NetworkedDouble m_networkedKV; // velo feed forward

  private TalonFXConfiguration m_activeConfig;

  /**
   * Drop-in constructor for default TalonFX motor
   *
   * @param id motor ID
   * @param canbus canbus name
   */
  public NetworkedTalonFX(int id, String canbus) {
    super(id, canbus);
    instanceCount++;

    m_name = "NetworkedTalonFX_" + instanceCount;
  }

  /**
   * Drop-in constructor for default TalonFX motor (using CANBus object)
   *
   * @param id motor ID
   * @param canbus CANBus object
   */
  public NetworkedTalonFX(int id, CANBus canbus) {
    super(id, canbus);
    instanceCount++;

    m_name = "NetworkedTalonFX_" + instanceCount;
  }

  /**
   * Constructor with mname setter
   *
   * @param id motor ID
   * @param canbus canbus name
   * @param name motor name for Network Tables
   */
  public NetworkedTalonFX(int id, String canbus, String name) {
    super(id, canbus);
    instanceCount++;

    m_name = name;
  }

  /**
   * Constructor with name setter and CANBus object support
   *
   * @param id motor ID
   * @param canbus CANBus object
   * @param name motor name for Network Tables
   */
  public NetworkedTalonFX(int id, CANBus canbus, String name) {
    super(id, canbus);
    instanceCount++;

    m_name = name;
  }

  private void setupNT(
      double kA, double kD, double kG, double kI, double kP, double kS, double kV) {
    m_networkedKA = new NetworkedDouble("/NetworkedLib/" + m_name + "/kA", kA);
    m_networkedkD = new NetworkedDouble("/NetworkedLib/" + m_name + "/kD", kD);
    m_networkedkG = new NetworkedDouble("/NetworkedLib/" + m_name + "/kG", kG);
    m_networkedkI = new NetworkedDouble("/NetworkedLib/" + m_name + "/kI", kI);
    m_networkedkP = new NetworkedDouble("/NetworkedLib/" + m_name + "/kP", kP);
    m_networkedKS = new NetworkedDouble("/NetworkedLib/" + m_name + "/kS", kS);
    m_networkedKV = new NetworkedDouble("/NetworkedLib/" + m_name + "/kV", kV);
  }

  /**
   * Use this function instead of Motor.getConfigurator().Apply(_)
   *
   * <p>Designed to apply the config and save the whole config for dynamic updates
   *
   * @param config The TalonFXConfiguration to apply
   */
  public void applyConfiguration(TalonFXConfiguration config) {
    m_activeConfig = config;

    this.getConfigurator().apply(config);
    setupNT(
        m_activeConfig.Slot0.kA,
        m_activeConfig.Slot0.kD,
        m_activeConfig.Slot0.kG,
        m_activeConfig.Slot0.kI,
        m_activeConfig.Slot0.kP,
        m_activeConfig.Slot0.kS,
        m_activeConfig.Slot0.kV);
  }

  /**
   * Call this in the periodic function to update the motor configs
   *
   * <p>THIS IS REQUIRED FOR THE DYNAMIC SLOT0 TO WORK!
   */
  public void periodic() {
    // checks if any networked doubles have new information
    if (m_networkedKA.available()
        || m_networkedkD.available()
        || m_networkedkG.available()
        || m_networkedkI.available()
        || m_networkedkP.available()
        || m_networkedKS.available()
        || m_networkedKV.available()) {

      // ensures values like gravity FF type and other are perserved
      m_activeConfig.Slot0 = m_activeConfig
          .Slot0
          .withKA(m_networkedKA.get())
          .withKD(m_networkedkD.get())
          .withKG(m_networkedkG.get())
          .withKI(m_networkedkI.get())
          .withKP(m_networkedkP.get())
          .withKS(m_networkedKS.get())
          .withKV(m_networkedKV.get());

      // the reason we arent directly applying slot0Configs is because it leaves open
      // more stuff to
      // add to the networked possiblities later.
      // in the future we could network other parts of the config (like current
      // limits!)

      this.getConfigurator().apply(m_activeConfig);
    }
  }
}
