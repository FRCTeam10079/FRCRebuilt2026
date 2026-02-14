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
  private DoubleTopic m_doubleTopic;
  private DoublePublisher m_doublePublisher;
  private DoubleSubscriber m_doubleSubscriber;

  private NetworkTableListener m_doubleListener;

  // this value is read and written to from multiple threads
  private volatile boolean m_newData = false;

  /**
   * Creates a networkedDouble object
   *
   * @param topicString the name of the topic
   * @param defaultValue the default double value to set the topic to
   */
  public NetworkedDouble(String topicString, double defaultValue) {
    NetworkTableInstance defaultNT = NetworkTableInstance.getDefault();
    m_doubleTopic = defaultNT.getDoubleTopic(topicString);

    m_doublePublisher = m_doubleTopic.publish();
    m_doubleSubscriber = m_doubleTopic.subscribe(defaultValue);

    m_doublePublisher.set(defaultValue);

    // ONLY detects REMOTE value updates
    // not designed to detect local code changes
    m_doubleListener = NetworkTableListener.createListener(
        m_doubleTopic,
        EnumSet.of(NetworkTableEvent.Kind.kValueRemote),
        (NetworkTableEvent event) -> {
          // legit just a lambda to make m_newData true
          // hopefully thread safe ♥
          m_newData = true;
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
    if (!m_newData) return false;
    m_newData = false;
    return true;
  }

  /** Closes active listeners */
  public void close() {
    m_doubleListener.close();
  }

  /**
   * Sets the value via the publisher
   *
   * @param value the value to set
   */
  public void set(double value) {
    m_doublePublisher.set(value);
  }

  /**
   * Gets the value from the subscriber
   *
   * @return the value recieved from the subscriber
   */
  public double get() {
    return m_doubleSubscriber.get();
  }
}
