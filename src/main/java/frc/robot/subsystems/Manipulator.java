// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import javax.swing.text.Position;

import org.ejml.ops.FConvertArrays;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.*;
import frc.robot.Constants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.utils.PID;

public class Manipulator extends SubsystemBase {
  /** Creates a new Manipulator. */
  private CANSparkMax intakeMotor;
  private CANSparkMax pivotMotor;

  private TalonFX telescopingMotor;
  private TalonFXConfiguration telescopingConfig;
  private SoftwareLimitSwitchConfigs telescopingLimitSwitchConfigs;
  
  private SparkMaxPIDController pivotPIDController;
  // private RelativeEncoder pivotEncoder;

  private SparkMaxPIDController telescopingPIDController; 

  private DigitalInput cubeSensor;
  private DigitalInput coneSensor;
  private double printlol = 0;
  private VoltageOut m_request;
  private double telescopingPos;
  private StatusSignal<Double> rotorPositionSignal;

  private SparkMaxAbsoluteEncoder absoluteEncoder;

  public Manipulator() {
    intakeMotor = new CANSparkMax(MotorConstants.INTAKE_MOTOR_ID, CANSparkMax.MotorType.kBrushless);

    cubeSensor = new DigitalInput(0);
    coneSensor = new DigitalInput(1);

    pivotMotor = new CANSparkMax(MotorConstants.PIVOT_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    pivotMotor.setInverted(true);
    pivotMotor.setOpenLoopRampRate(0.2);
    pivotMotor.setClosedLoopRampRate(0.2);
    pivotMotor.setSoftLimit(SoftLimitDirection.kReverse, 0.29f);
    pivotMotor.setSoftLimit(SoftLimitDirection.kForward, 0); //shawn is doo doo

    telescopingMotor = new TalonFX(MotorConstants.TELE_ARM_MOTOR_ID);
    absoluteEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    absoluteEncoder.setInverted(true);

    pivotPIDController = pivotMotor.getPIDController();
    pivotPIDController.setFeedbackDevice(absoluteEncoder);
    pivotPIDController.setP(1.2);
    pivotPIDController.setI(0.0);
    pivotPIDController.setD(0.08);
    pivotPIDController.setFF(0.01);
    pivotPIDController.setPositionPIDWrappingEnabled(true);
    pivotPIDController.setPositionPIDWrappingMaxInput(1);
    pivotPIDController.setPositionPIDWrappingMinInput(0);
    pivotPIDController.setOutputRange(-0.45, 0.45);

    // TO DO add soft limits
    telescopingMotor.setRotorPosition(0);


    // Telescpoing Arm
    Slot0Configs teleSlot0configs = new Slot0Configs();
    telescopingConfig = new TalonFXConfiguration();
    telescopingLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
    // telescopingLimitSwitchConfigs.ForwardSoftLimitEnable = true;  
    // telescopingLimitSwitchConfigs.ForwardSoftLimitThreshold = 2;
    // telescopingLimitSwitchConfigs.ReverseSoftLimitEnable = true;
    // telescopingLimitSwitchConfigs.ReverseSoftLimitThreshold = -240;


    telescopingConfig.SoftwareLimitSwitch = telescopingLimitSwitchConfigs;
    telescopingMotor.getConfigurator().apply(telescopingConfig);
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
    telescopingPos = -rotorPositionSignal.getValue();
    // Gear ratio of pivot 150 to 1
    SmartDashboard.putBoolean("Cone Sensor", coneSensor.get());
    SmartDashboard.putBoolean("Cube Sensor", cubeSensor.get());
    SmartDashboard.putNumber("Pivot Encoderr", absoluteEncoder.getPosition());
    SmartDashboard.putNumber("Telescoping Encoderr", telescopingPos);
    SmartDashboard.putNumber("ScorePos", Constants.ManipulatorConstants.scoring_pos);

    // SmartDashboard.putNumber("Telescoping Current", telescopingMotor.getStatorCurrent().getValue());
    
  }


  public void jessciaZero(){
    ManipulatorConstants.scoring_pos = 0;
  }

  public void jessciaOne(){
    ManipulatorConstants.scoring_pos = 0;
  }

  public void jessciaTwo(){
    ManipulatorConstants.scoring_pos = 0;
  }

  public void jessiaThree(){

  }

  public double getPivotEncoder() {
    return absoluteEncoder.getPosition();
  }

  public double getArmEncoder() {
    return telescopingPos;
  }

  // Intake Functions
  public void cubeIntake() {
    intakeMotor.set(.8);
  }

  public void coneIntake() {
    // if (coneSensor.get()){
      // intakeMotor.set(0);
    // }
    // else {
    intakeMotor.set(-0.6);
    // }
  }

  public void holdCone(){
    intakeMotor.set(-0.1);
  }

  public void cubeScore() {

    intakeMotor.set(-0.4);
  }

  public void coneScore() {
    intakeMotor.set(0.4); // 0.238092005252838
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
    pivotPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
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

  public void pivotGround(){
    pivotPIDController.setReference(ManipulatorConstants.PIVOT_GROUND_INTAKE_POSITION, CANSparkMax.ControlType.kPosition);
  }

  // Telescoping Functions

  public void setTelescopingSpeed(double speed){
    telescopingMotor.setControl(m_request.withOutput(speed));
  }

  public void setTelescopingPosition(double position) {
    // telescopingMotor.setRotorPosition(position); // does this work?
    telescopingMotor.setControl(new PositionDutyCycle(position));
  }

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