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
import frc.robot.utils.PID;

public class Pivot extends SubsystemBase {
  /** Creates a new Pivot. */

  private CANSparkMax pivotMotor;
  private SparkMaxPIDController pivotPIDController;
  private RelativeEncoder pivotEncoder;

  public Pivot() {
  
    pivotMotor = new CANSparkMax(Constants.ManipulatorConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);

    pivotPIDController = pivotMotor.getPIDController();
    pivotEncoder = pivotMotor.getEncoder();

    pivotPIDController.setP(Constants.ManipulatorConstants.pivotPID.p);
    pivotPIDController.setI(Constants.ManipulatorConstants.pivotPID.i);
    pivotPIDController.setD(Constants.ManipulatorConstants.pivotPID.d);

  }

  public void initPIDController(PID pid){

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
