// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.ctre.phoenix.unmanaged.Unmanaged;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.WRIST;
import frc.robot.Constants.WRIST.THRESHOLD;
import frc.robot.subsystems.StateHandler;

/** Add your docs here. */
public class WristIOSim extends WristIOReal{

    private static int m_simEncoderSign = 1;

    // Mech2d setup
    private final MechanismLigament2d m_wristGearboxLigament2d =
    new MechanismLigament2d("FourbarGearbox", WRIST.fourbarGearboxHeight, 90);
    private final MechanismLigament2d m_wristLigament2d =
    new MechanismLigament2d("Fourbar", WRIST.length, WRIST.fourbarAngleDegrees);

    // Simulation setup
    private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          WRIST.gearBox,
          WRIST.gearRatio,
          SingleJointedArmSim.estimateMOI(WRIST.length, WRIST.mass),
          WRIST.length,
          THRESHOLD.ABSOLUTE_MIN.get(),
          THRESHOLD.ABSOLUTE_MAX.get(),
          false
          // VecBuilder.fill(2.0 * Math.PI / 2048.0) // Add noise with a std-dev of 1 tick
          );

    public WristIOSim() {
        super();

        m_simEncoderSign = wristMotor.getInverted() ? -1 : 1;

        m_wristLigament2d.setColor(new Color8Bit(144, 238, 144)); // Light green
    }

    public MechanismLigament2d getGearboxLigament() {
        return m_wristGearboxLigament2d;
      }
    
      public MechanismLigament2d getWristLigament() {
        return m_wristLigament2d;
      }

          @Override
          public void simulationPeriodic() {
            m_armSim.setInputVoltage(MathUtil.clamp(wristMotor.getMotorOutputVoltage(), -12, 12));
        
            double dt = StateHandler.getSimDt();
            m_armSim.update(dt);
        
            Unmanaged.feedEnable(20);
        
            // Using negative sensor units to match physical behavior
            wristMotor
                .getSimCollection()
                .setIntegratedSensorRawPosition(
                    (int)
                        (m_simEncoderSign
                            * Units.radiansToDegrees(m_armSim.getAngleRads())
                            / WRIST.encoderUnitsToDegrees));
        
            wristMotor
                .getSimCollection()
                .setIntegratedSensorVelocity(
                    (int)
                        (m_simEncoderSign
                            * Units.radiansToDegrees(m_armSim.getVelocityRadPerSec())
                            / WRIST.encoderUnitsToDegrees
                            * 10.0));
        
            wristMotor.getSimCollection().setBusVoltage(RobotController.getBatteryVoltage());
          }
}
