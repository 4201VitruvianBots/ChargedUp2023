package frc.robot.utils.logging;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;

/**
 * A wrapper for DoublePublisher that automatically publishes to AdvantageKit.
 * Removes whitespace from key to be compatible with AdvantageKit naming standards.
 */
public class AdvantageDoublePublisher {
    DoublePublisher m_publisher;
    String m_path;
    
    public void publish(NetworkTable tab, String topicName) {
        m_publisher = tab.getDoubleTopic(topicName).publish();
        m_path = (tab.getPath() + "/" + topicName).replaceAll("\\s+","");
    }

    public DoublePublisher getPublisher() {
        return m_publisher;
    }

    public void set(double value) {
        m_publisher.set(value);
        Logger.getInstance().recordOutput("LoggingUtils" + m_path, value);
    }
}