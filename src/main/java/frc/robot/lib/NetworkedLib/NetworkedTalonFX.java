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

  private String name;

  // networked prefix to just to avoid ambigiouity between actual values and their networked
  // partners
  private NetworkedDouble networkedKA; // acceleration gain
  private NetworkedDouble networkedKD; // derivative gain
  private NetworkedDouble networkedKG; // gravity gain
  private NetworkedDouble networkedKI; // integral gain
  private NetworkedDouble networkedKP; // proprotional gain
  private NetworkedDouble networkedKS; // static constant (static friction)
  private NetworkedDouble networkedKV; // velo feed forward

  private TalonFXConfiguration activeConfig;

  /**
   * Drop-in constructor for default TalonFX motor
   *
   * @param ID motor ID
   * @param canbus canbus name
   */
  public NetworkedTalonFX(int ID, String canbus) {
    super(ID, canbus);
    instanceCount++;

    name = "NetworkedTalonFX_" + instanceCount;
  }

  /**
   * Drop-in constructor for default TalonFX motor (using CANBus object)
   *
   * @param ID motor ID
   * @param canbus CANBus object
   */
  public NetworkedTalonFX(int ID, CANBus canbus) {
    super(ID, canbus);
    instanceCount++;

    name = "NetworkedTalonFX_" + instanceCount;
  }

  /**
   * Constructor with name setter
   *
   * @param ID motor ID
   * @param canbus CAN bus name
   * @param name motor name for Network Tables
   * @deprecated Constructing devices with a CAN bus string is deprecated for removal in the 2027
   *     season. Construct devices using a {@link CANBus} instance instead.
   */
  @Deprecated(since = "2026", forRemoval = true)
  public NetworkedTalonFX(int ID, String canbus, String name) {
    this(ID, new CANBus(canbus), name);
    instanceCount++;

    this.name = name;
  }

  /**
   * Constructor with name setter and CANBus object support
   *
   * @param ID motor ID
   * @param canbus CANBus object
   * @param name motor name for Network Tables
   */
  public NetworkedTalonFX(int ID, CANBus canbus, String name) {
    super(ID, canbus);
    instanceCount++;

    this.name = name;
  }

  private void setupNT(
      double kA, double kD, double kG, double kI, double kP, double kS, double kV) {
    networkedKA = new NetworkedDouble("/NetworkedLib/" + name + "/kA", kA);
    networkedKD = new NetworkedDouble("/NetworkedLib/" + name + "/kD", kD);
    networkedKG = new NetworkedDouble("/NetworkedLib/" + name + "/kG", kG);
    networkedKI = new NetworkedDouble("/NetworkedLib/" + name + "/kI", kI);
    networkedKP = new NetworkedDouble("/NetworkedLib/" + name + "/kP", kP);
    networkedKS = new NetworkedDouble("/NetworkedLib/" + name + "/kS", kS);
    networkedKV = new NetworkedDouble("/NetworkedLib/" + name + "/kV", kV);
  }

  /**
   * Use this function instead of Motor.getConfigurator().Apply(_)
   *
   * <p>Designed to apply the config and save the whole config for dynamic updates
   *
   * @param config The TalonFXConfiguration to apply
   */
  public void applyConfiguration(TalonFXConfiguration config) {
    activeConfig = config;

    getConfigurator().apply(config);
    setupNT(
        activeConfig.Slot0.kA,
        activeConfig.Slot0.kD,
        activeConfig.Slot0.kG,
        activeConfig.Slot0.kI,
        activeConfig.Slot0.kP,
        activeConfig.Slot0.kS,
        activeConfig.Slot0.kV);
  }

  /**
   * Call this in the periodic function to update the motor configs
   *
   * <p>THIS IS REQUIRED FOR THE DYNAMIC SLOT0 TO WORK!
   */
  public void periodic() {
    // checks if any networked doubles have new information
    if (networkedKA.available()
        || networkedKD.available()
        || networkedKG.available()
        || networkedKI.available()
        || networkedKP.available()
        || networkedKS.available()
        || networkedKV.available()) {

      // ensures values like gravity FF type and other are preserved
      activeConfig.Slot0 = activeConfig
          .Slot0
          .withKA(networkedKA.get())
          .withKD(networkedKD.get())
          .withKG(networkedKG.get())
          .withKI(networkedKI.get())
          .withKP(networkedKP.get())
          .withKS(networkedKS.get())
          .withKV(networkedKV.get());

      // The reason we aren't directly applying slot0Configs is that it leaves open more stuff to
      // add to the networked possibilities later.
      // In the future we could network other parts of the config (like current limits).

      getConfigurator().apply(activeConfig);
    }
  }
}
