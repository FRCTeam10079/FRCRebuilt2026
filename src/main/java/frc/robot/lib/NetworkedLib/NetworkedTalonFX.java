package frc.robot.lib.NetworkedLib;

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

  // networked_ prefix to just to avoid ambigiouity between actual values and their networked
  // partners
  private NetworkedDouble Networked_kA; // derivative gain
  private NetworkedDouble Networked_kD; // derivative gain
  private NetworkedDouble Networked_kG; // derivative gain
  private NetworkedDouble Networked_kI; // integral gain
  private NetworkedDouble Networked_kP; // proprotional gain
  private NetworkedDouble Networked_kS; // static constant (static friction)
  private NetworkedDouble Networked_kV; // velo feed forward

  private TalonFXConfiguration activeConfig;

  /**
   * Drop-in constructor for default TalonFX motor
   *
   * @param ID motor ID
   * @param canbus canbus name (no support for new CANBus object)
   */
  public NetworkedTalonFX(int ID, String canbus) {
    super(ID, canbus);
    instanceCount++;

    this.name = "NetworkedTalonFX_" + instanceCount;
  }

  /**
   * Constructor with name setter
   *
   * @param ID motor ID
   * @param canbus canbus name
   * @param name motor name for Network Tables
   */
  public NetworkedTalonFX(int ID, String canbus, String name) {
    super(ID, canbus);
    instanceCount++;

    this.name = name;
  }

  private void setupNT(
      double kA, double kD, double kG, double kI, double kP, double kS, double kV) {
    this.Networked_kA = new NetworkedDouble("/NetworkedLib/" + this.name + "/kA", kA);
    this.Networked_kD = new NetworkedDouble("/NetworkedLib/" + this.name + "/kD", kD);
    this.Networked_kG = new NetworkedDouble("/NetworkedLib/" + this.name + "/kG", kG);
    this.Networked_kI = new NetworkedDouble("/NetworkedLib/" + this.name + "/kI", kI);
    this.Networked_kP = new NetworkedDouble("/NetworkedLib/" + this.name + "/kP", kP);
    this.Networked_kS = new NetworkedDouble("/NetworkedLib/" + this.name + "/kS", kS);
    this.Networked_kV = new NetworkedDouble("/NetworkedLib/" + this.name + "/kV", kV);
  }

  /**
   * Use this function instead of Motor.getConfigurator().Apply(_)
   *
   * <p>Designed to apply the config and save the Slot0 for dynamic updates
   *
   * @param config The TalonFXConfiguration to apply
   */
  public void applyConfiguration(TalonFXConfiguration config) {
    this.activeConfig = config;

    this.getConfigurator().apply(config);
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
   * Call this in the periodic function to update the motor configs THIS IS REQUIRED FOR THE DYNAMIC
   * SLOT0 TO WORK!
   */
  public void periodic() {
    // im not smart enough to think of an easy way to avoid this nesting (i think its fine just this
    // once!)
    if (Networked_kA.available()
        || Networked_kD.available()
        || Networked_kG.available()
        || Networked_kI.available()
        || Networked_kP.available()
        || Networked_kS.available()
        || Networked_kV.available()) {
      // ensures values like gravity FF type and other are perserved
      this.activeConfig.Slot0 = this.activeConfig
          .Slot0
          .withKA(Networked_kA.get())
          .withKD(Networked_kD.get())
          .withKG(Networked_kG.get())
          .withKI(Networked_kI.get())
          .withKP(Networked_kP.get())
          .withKS(Networked_kS.get())
          .withKV(Networked_kV.get());

      // the reason we arent directly applying slot0Configs is because it leaves open more stuff to
      // add to the networked possiblities later
      // in the future we could network other parts of the config (like current limits!)

      this.getConfigurator().apply(this.activeConfig);
    }
  }
}
