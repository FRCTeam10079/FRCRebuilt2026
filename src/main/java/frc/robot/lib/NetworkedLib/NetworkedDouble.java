package frc.robot.lib.NetworkedLib;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import java.util.EnumSet;

/*
 * Simplifies usage of doubles in NT
 * Used in this project to simplify code
 *
 * I didn't realize til after making this but NetworkTableEntry might work better here
 */
public class NetworkedDouble {
  private DoubleTopic doubleTopic;
  private DoublePublisher doublePublisher;
  private DoubleSubscriber doubleSubscriber;

  private boolean newData = false;

  public NetworkedDouble(String topicString, double defaultValue) {
    NetworkTableInstance defaultNT = NetworkTableInstance.getDefault();
    this.doubleTopic = defaultNT.getDoubleTopic(topicString);

    this.doublePublisher = doubleTopic.publish();
    this.doubleSubscriber = doubleTopic.subscribe(defaultValue);

    this.doublePublisher.set(defaultValue);

    // ONLY detects REMOTE value updates
    // not designed to detect local code changes
    NetworkTableListener.createListener(
        doubleTopic, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), (NetworkTableEvent event) -> {
          // legit just a lambda to make newData true
          // hopefully thread safe ♥
          this.newData = true;
        });
  }

  /**
   * Returns whether or not new data is available
   *
   * <p>Once this function returns true, it will not return true until new data is found
   *
   * @return
   */
  public boolean available() {
    if (!this.newData) {
      return false;
    }
    this.newData = false;
    return true;
  }

  /**
   * Sets the value via the publisher
   *
   * @param value
   */
  public void set(double value) {
    this.doublePublisher.set(value);
  }

  /**
   * Gets the value from the subscriber
   *
   * @return
   */
  public double get() {
    return this.doubleSubscriber.get();
  }
}
