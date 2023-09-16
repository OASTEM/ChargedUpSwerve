// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Manipulator;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TelescopingArm extends SubsystemBase {
  /** Creates a new TelescopingArm. */
  private CANSparkMax motor;
  public TelescopingArm() {
    motor = new CANSparkMax(Constants.ManipulatorConstants.TELESCOPING_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
