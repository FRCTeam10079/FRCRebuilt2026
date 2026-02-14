package frc.robot.lib.NetworkedLib;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import java.util.EnumSet;
import java.util.concurrent.atomic.AtomicBoolean;

/*
 * Simplifies usage of doubles in NT
 * Used in this project to simplify code
 *
 * I didn't realize til after making this but NetworkTableEntry might work better here
 */
public class NetworkedDouble {
  private final DoublePublisher doublePublisher;
  private final DoubleSubscriber doubleSubscriber;

  private final NetworkTableListener doubleListener;

  private final AtomicBoolean newData = new AtomicBoolean(false);

  /**
   * @param topicString The name of the topic
   * @param defaultValue The default double value to set the topic to
   */
  public NetworkedDouble(String topicString, double defaultValue) {
    DoubleTopic doubleTopic = NetworkTableInstance.getDefault().getDoubleTopic(topicString);

    doublePublisher = doubleTopic.publish();
    doubleSubscriber = doubleTopic.subscribe(defaultValue);

    doublePublisher.set(defaultValue);

    // ONLY detects REMOTE value updates
    // not designed to detect local code changes
    doubleListener = NetworkTableListener.createListener(
        doubleTopic,
        EnumSet.of(NetworkTableEvent.Kind.kValueRemote),
        (NetworkTableEvent event) -> newData.set(true));
  }

  /**
   * Get whether new data is available. <br>
   * Once this function returns true, it will not return true until new data is found
   *
   * @return Whether new data is available since the last time it was called.
   */
  public boolean available() {
    return newData.getAndSet(false);
  }

  /** Closes active listeners */
  public void close() {
    doubleListener.close();
  }

  /**
   * Sets the value via the publisher
   *
   * @param value the value to set
   */
  public void set(double value) {
    doublePublisher.set(value);
  }

  /**
   * Gets the value from the subscriber.
   *
   * @return The value received from the subscriber
   */
  public double get() {
    return doubleSubscriber.get();
  }
}
