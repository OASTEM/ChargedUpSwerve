// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import javax.swing.text.Position;

import org.ejml.ops.FConvertArrays;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.*;
import frc.robot.utils.PID;

public class Manipulator extends SubsystemBase {
  /** Creates a new Manipulator. */
  private CANSparkMax intakeMotor;
  private CANSparkMax pivotMotor;

  private TalonFX telescopingMotor;
  
  private SparkMaxPIDController pivotPIDController;
  private RelativeEncoder pivotEncoder;

  private SparkMaxPIDController telescopingPIDController; 

  private DigitalInput coneSensor;
  private DigitalInput cubeSensor;
  private double printlol = 0;
  private VoltageOut m_request;
  private double telescopingPos;
  private StatusSignal<Double> rotorPositionSignal;

  private SparkMaxAbsoluteEncoder absoluteEncoder;

  public Manipulator() {
    // intakeMotor = new CANSparkMax(6, CANSparkMax.MotorType.kBrushless);

    // coneSensor = new DigitalInput(8);
    // cubeSensor = new DigitalInput(9);
  
    pivotMotor = new CANSparkMax(14, CANSparkMax.MotorType.kBrushless);
    pivotMotor.setInverted(true);
    pivotMotor.setOpenLoopRampRate(0.2);
    pivotMotor.setClosedLoopRampRate(0.2);

    telescopingMotor = new TalonFX(13);
    absoluteEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    absoluteEncoder.setInverted(false);

    pivotPIDController = pivotMotor.getPIDController();
    pivotPIDController.setFeedbackDevice(absoluteEncoder);
    pivotPIDController.setP(1.2);
    pivotPIDController.setI(0.0);
    pivotPIDController.setD(0.08);
    pivotPIDController.setPositionPIDWrappingEnabled(true);
    pivotPIDController.setPositionPIDWrappingMaxInput(1);
    pivotPIDController.setPositionPIDWrappingMinInput(0);
    pivotPIDController.setOutputRange(-0.25, 0.25);

    // TO DO add soft limits
    telescopingMotor.setRotorPosition(0);

    // Telescpoing Arm
    Slot0Configs teleSlot0configs = new Slot0Configs();
    m_request = new VoltageOut(0);
    

    teleSlot0configs.kP = 0.01;
    teleSlot0configs.kI = 0;
    teleSlot0configs.kD = 0;


    // telescopingMotor.configPeakOutputForward(0.5);
    // telescopingMotor.configPeakOutputReverse(-0.5);

    telescopingMotor.getConfigurator().apply(teleSlot0configs, 0.5);

  }

  @Override
  public void periodic() {
    rotorPositionSignal = telescopingMotor.getRotorPosition();
    telescopingPos = rotorPositionSignal.getValue();
    // Gear ratio of pivot 150 to 1
    // SmartDashboard.putBoolean("Cone Sensor", coneSensor.get());
    // SmartDashboard.putBoolean("Cube Sensor", cubeSensor.get());\
    SmartDashboard.putNumber("Pivot Encoder", absoluteEncoder.getPosition());
    SmartDashboard.putNumber("Telescoping Encoder", telescopingPos);

    // SmartDashboard.putNumber("Telescoping Current", telescopingMotor.getStatorCurrent().getValue());
    
  }

  // Intake Functions
  public void cubeIntake() {
    intakeMotor.set(0.1);
  }

  public void coneIntake() {
    intakeMotor.set(-0.2);
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  // public void calibrateTele() {
  //   while (Math.abs(telescopingMotor.getStatorCurrent()) < 4) {
  //     telescopingMotor.set(ControlMode.PercentOutput, 0.15);
  //   }
  //   if (Math.abs(telescopingMotor.getStatorCurrent()) > 4) {
  //     telescopingMotor.set(ControlMode.PercentOutput, 0);
  //   }
  // }


  // Pivot Functions
  public void setPivotPosition(double position) {
    pivotEncoder.setPosition(position);
  }

  public void setPivotSpeed(double speed){
    pivotMotor.set(speed);
  }

  public void initPivotPIDController(PID pid) {
    pivotPIDController.setP(pid.p);
    pivotPIDController.setI(pid.i);
    pivotPIDController.setD(pid.d);
  }

  public void calibratePivot(){
    pivotPIDController.setReference(0.10, CANSparkMax.ControlType.kPosition);
  }

  public void holdPivot(){
    pivotPIDController.setReference(absoluteEncoder.getPosition(), CANSparkMax.ControlType.kPosition);
  }
  // Telescoping Functions

  public void setTelescopingSpeed(double speed){
    telescopingMotor.setControl(m_request.withOutput(speed));
  }

  // public void setTelescopingPosition(double position) {
  //   telescopingMotor.set(ControlMode.Position, 10);
  // }

  public void initTelescopingPIDController(PID pid) {
    telescopingPIDController.setP(pid.p);
    telescopingPIDController.setI(pid.i);
    telescopingPIDController.setD(pid.d);
  }

  // public double getTelescopingStatorCurrent(){
  //   return telescopingMotor.getStatorCurrent();
  // }

  // public void resetTelescopingEncoder(){
  //   telescopingMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
  // }

  // sensor functions
  public boolean getConeSensor() {
    return coneSensor.get();
  }

  public boolean getCubeSensor() {
    return cubeSensor.get();
  }
}