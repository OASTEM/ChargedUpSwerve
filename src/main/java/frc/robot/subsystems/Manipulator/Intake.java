// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Manipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private CANSparkMax intakeMotor;


  public Intake() {
    intakeMotor = new CANSparkMax(Constants.ManipulatorConstants.INTAKE_MOTOR_ID, MotorType.kBrushless); // MotorType.kBrushed
  }

  public void spinIntake(double speed) {
    intakeMotor.set(speed);
  }

  public void spinIntake() {
    intakeMotor.set(Constants.ManipulatorConstants.INTAKE_SPEED);
  }
  
  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
