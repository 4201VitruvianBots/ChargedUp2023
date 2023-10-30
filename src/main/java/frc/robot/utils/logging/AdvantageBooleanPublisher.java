package frc.robot.utils.logging;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;

/**
 * A wrapper for BooleanPublisher that automatically publishes to AdvantageKit.
 * Removes whitespace from key to be compatible with AdvantageKit naming standards.
 */
public class AdvantageBooleanPublisher {
    BooleanPublisher m_publisher;
    String m_path;

    public void publish(NetworkTable tab, String topicName) {
        m_publisher = tab.getBooleanTopic(topicName).publish();
        m_path = (tab.getPath() + "/" + topicName).replaceAll("\\s+","");
    }

    public BooleanPublisher getPublisher() {
        return m_publisher;
    }

    public void set(Boolean value) {
        m_publisher.set(value);
        Logger.getInstance().recordOutput("LoggingUtils" + m_path, value);
    }
}