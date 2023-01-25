package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoBalance extends SubsystemBase {

  private PIDController m_xController = new PIDController(0.8, 0, 0);

  Pigeon2 gyroPigeon2 = new Pigeon2(9);

  public AutoBalance() {}

  public double getPitch() {
    return gyroPigeon2.getPitch();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Robot Pitch", getPitch());

    double output = m_xController.calculate(getPitch());

    SmartDashboard.putNumber("output", output);
  }
}
