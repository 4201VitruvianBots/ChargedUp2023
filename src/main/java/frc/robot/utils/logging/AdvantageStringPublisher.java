package frc.robot.utils.logging;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringPublisher;

/**
 * A wrapper for StringPublisher that automatically publishes to AdvantageKit.
 * Removes whitespace from key to be compatible with AdvantageKit naming standards.
 */
public class AdvantageStringPublisher {
    StringPublisher m_publisher;
    String m_path;

    public void publish(NetworkTable tab, String topicName) {
        m_publisher = tab.getStringTopic(topicName).publish();
        m_path = (tab.getPath() + "/" + topicName).replaceAll("\\s+","");
    }

    public StringPublisher getPublisher() {
        return m_publisher;
    }

    public void set(String value) {
        m_publisher.set(value);
        Logger.getInstance().recordOutput("LoggingUtils" + m_path, value);
    }
}