package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import org.junit.jupiter.api.Test;
import utils.TestUtils;

import static utils.TestUtils.getPrivateObject;

public class FieldSimTest {
    protected RobotContainer m_robotContainer = new RobotContainer();
    protected SwerveDrive m_swerveDrive = m_robotContainer.getSwerveDrive();
    protected FieldSim m_fieldSim = m_robotContainer.getFieldSim();

    @Test
    public void FieldSimTest() {
        testRedAllianceRedNodes();
        testRedAllianceBlueCooperatitionNodes();
        testBlueAllianceBlueNodes();
        testBlueALlianceRedCooperatitionNodes();
    }

    @Test
    public void testRedAllianceRedNodes() {
        TestUtils.setPrivateField(m_swerveDrive, "m_currentAlliance", DriverStation.Alliance.Red);
        m_swerveDrive.setOdometry(new Pose2d(SimConstants.fieldLength, 0, Rotation2d.fromDegrees(0)));
        m_fieldSim.updateValidNodes(Constants.SCORING_STATE.LOW);
        assert(m_fieldSim.getValidNodes().size() == 9);
        for(var node:m_fieldSim.getValidNodes()) {
            assert(node.getX() > SimConstants.fieldLength / 2);
        }
        m_fieldSim.updateValidNodes(Constants.SCORING_STATE.MID_CONE);
        assert(m_fieldSim.getValidNodes().size() == 6);
        for(var node:m_fieldSim.getValidNodes()) {
            assert(node.getX() > SimConstants.fieldLength / 2);
        }
        m_fieldSim.updateValidNodes(Constants.SCORING_STATE.MID_CUBE);
        assert(m_fieldSim.getValidNodes().size() == 3);
        for(var node:m_fieldSim.getValidNodes()) {
            assert(node.getX() > SimConstants.fieldLength / 2);
        }
    }

    @Test
    public void testRedAllianceBlueCooperatitionNodes() {
        TestUtils.setPrivateField(m_fieldSim, "m_currentAlliance", DriverStation.Alliance.Red);
        m_swerveDrive.setOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        m_fieldSim.updateValidNodes(Constants.SCORING_STATE.LOW);
        assert(m_fieldSim.getValidNodes().size() == 3);
        for(var node:m_fieldSim.getValidNodes()) {
            assert(node.getX() < SimConstants.fieldLength / 2);
        }
        m_fieldSim.updateValidNodes(Constants.SCORING_STATE.MID_CONE);
        assert(m_fieldSim.getValidNodes().size() == 2);
        for(var node:m_fieldSim.getValidNodes()) {
            assert(node.getX() < SimConstants.fieldLength / 2);
        }
        m_fieldSim.updateValidNodes(Constants.SCORING_STATE.MID_CUBE);
        assert(m_fieldSim.getValidNodes().size() == 1);
        for(var node:m_fieldSim.getValidNodes()) {
            assert(node.getX() < SimConstants.fieldLength / 2);
        }
    }

    @Test
    public void testBlueAllianceBlueNodes() {
        TestUtils.setPrivateField(m_fieldSim, "m_currentAlliance", DriverStation.Alliance.Blue);
        m_swerveDrive.setOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        m_fieldSim.updateValidNodes(Constants.SCORING_STATE.LOW);
        assert(m_fieldSim.getValidNodes().size() == 9);
        for(var node:m_fieldSim.getValidNodes()) {
            assert(node.getX() < SimConstants.fieldLength / 2);
        }
        m_fieldSim.updateValidNodes(Constants.SCORING_STATE.MID_CONE);
        assert(m_fieldSim.getValidNodes().size() == 6);
        for(var node:m_fieldSim.getValidNodes()) {
            assert(node.getX() < SimConstants.fieldLength / 2);
        }
        m_fieldSim.updateValidNodes(Constants.SCORING_STATE.MID_CUBE);
        assert(m_fieldSim.getValidNodes().size() == 3);
        for(var node:m_fieldSim.getValidNodes()) {
            assert(node.getX() < SimConstants.fieldLength / 2);
        }
    }

    @Test
    public void testBlueALlianceRedCooperatitionNodes() {
        TestUtils.setPrivateField(m_fieldSim, "m_currentAlliance", DriverStation.Alliance.Blue);
        m_swerveDrive.setOdometry(new Pose2d(SimConstants.fieldLength, 0, Rotation2d.fromDegrees(0)));
        m_fieldSim.updateValidNodes(Constants.SCORING_STATE.LOW);
        assert(m_fieldSim.getValidNodes().size() == 3);
        for(var node:m_fieldSim.getValidNodes()) {
            assert(node.getX() > SimConstants.fieldLength / 2);
        }
        m_fieldSim.updateValidNodes(Constants.SCORING_STATE.MID_CONE);
        assert(m_fieldSim.getValidNodes().size() == 2);
        for(var node:m_fieldSim.getValidNodes()) {
            assert(node.getX() > SimConstants.fieldLength / 2);
        }
        m_fieldSim.updateValidNodes(Constants.SCORING_STATE.MID_CUBE);
        assert(m_fieldSim.getValidNodes().size() == 1);
        for(var node:m_fieldSim.getValidNodes()) {
            assert(node.getX() > SimConstants.fieldLength / 2);
        }
    }
}
