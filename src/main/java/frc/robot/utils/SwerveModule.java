// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class SwerveModule {
  private TalonFX driveMotor;
  private TalonFX steerMotor;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveId, int steerId) {
    driveMotor = new TalonFX(driveId);
    steerMotor = new TalonFX(steerId);

    driveMotor.configFactoryDefault();
    steerMotor.configFactoryDefault();

    driveMotor.setNeutralMode(NeutralMode.Brake);
    steerMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void setDriveSpeed(double speed) {
    driveMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setSteerSpeed(double speed) {
    steerMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setDrivePosition(double position) {
    driveMotor.set(ControlMode.Position, position);
  }

  public void setSteerPosition(double position) {
    steerMotor.set(ControlMode.Position, position);
  }
}
