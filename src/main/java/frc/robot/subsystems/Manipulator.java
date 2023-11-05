// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
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
  private TalonFXConfigurator teleConfigurator;
  private MotorOutputConfigs motorOutputConfigs;
  private SoftwareLimitSwitchConfigs telescopingLimitSwitchConfigs;
  
  private SparkMaxPIDController pivotPIDController;


  // private DigitalInput cubeSensor;
  // private DigitalInput coneSensor;
  private double printlol = 0;
  private VoltageOut m_request;
  private double telescopingPos;
  private StatusSignal<Double> rotorPositionSignal;

  private SparkMaxAbsoluteEncoder absoluteEncoder;

  public Manipulator() {
    intakeMotor = new CANSparkMax(MotorConstants.INTAKE_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // cubeSensor = new DigitalInput(0);
    // coneSensor = new DigitalInput(1);

    pivotMotor = new CANSparkMax(MotorConstants.PIVOT_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    absoluteEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    absoluteEncoder.setInverted(true);
    absoluteEncoder.setZeroOffset(0.67);
    pivotMotor.setInverted(true);
    pivotMotor.setOpenLoopRampRate(0.4);
    pivotMotor.setClosedLoopRampRate(0.4);
    pivotMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    pivotMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    pivotMotor.setSoftLimit(SoftLimitDirection.kForward, 0.40f);
    pivotMotor.setSoftLimit(SoftLimitDirection.kReverse, 0.1f); //shawn is doo doo
    telescopingMotor = new TalonFX(MotorConstants.TELE_ARM_MOTOR_ID);

    pivotPIDController = pivotMotor.getPIDController();
    pivotPIDController.setFeedbackDevice(absoluteEncoder);
    pivotPIDController.setP(7);
    pivotPIDController.setI(0.001);
    pivotPIDController.setD(0.0);
    pivotPIDController.setFF(0);
    pivotPIDController.setPositionPIDWrappingEnabled(true);
    pivotPIDController.setPositionPIDWrappingMaxInput(1);
    pivotPIDController.setPositionPIDWrappingMinInput(0);
    pivotPIDController.setOutputRange(-0.45, 0.45);

    // TO DO add soft limits
    telescopingMotor.setRotorPosition(0);

    // Telescpoing Arm
    Slot0Configs teleSlot0configs = new Slot0Configs();
    telescopingConfig = new TalonFXConfiguration();
    motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;


    telescopingLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
      telescopingLimitSwitchConfigs.ForwardSoftLimitEnable = true;  
      telescopingLimitSwitchConfigs.ForwardSoftLimitThreshold = ManipulatorConstants.TELESCOPING_SOFT_LIMIT_FORWARD;
      telescopingLimitSwitchConfigs.ReverseSoftLimitEnable = true;
      telescopingLimitSwitchConfigs.ReverseSoftLimitThreshold = ManipulatorConstants.TELESCOPING_SOFT_LIMIT_REVERSE;


    telescopingConfig.SoftwareLimitSwitch = telescopingLimitSwitchConfigs;
    telescopingMotor.getConfigurator().apply(telescopingConfig);
    telescopingMotor.getConfigurator().apply(motorOutputConfigs);
    m_request = new VoltageOut(0);
    
    teleSlot0configs.kP = 0.06;
    teleSlot0configs.kI = 0.00001;
    teleSlot0configs.kD = 0;


    // telescopingMotor.configPeakOutputForward(0.5);
    // telescopingMotor.configPeakOutputReverse(-0.5);

    telescopingMotor.getConfigurator().apply(teleSlot0configs, 0.5);

  }

  @Override
  public void periodic() {
    rotorPositionSignal = telescopingMotor.getRotorPosition();
    telescopingPos = rotorPositionSignal.getValue();

    // SmartDashboard.putBoolean("Cone Sensor", coneSensor.get());
    // SmartDashboard.putBoolean("Cube Sensor", cubeSensor.get());
    SmartDashboard.putNumber("Pivot Encoderr", absoluteEncoder.getPosition());
    SmartDashboard.putNumber("Telescoping Encoderr", telescopingPos);
    SmartDashboard.putNumber("ScorePos", Constants.ManipulatorConstants.scoring_pos);
    SmartDashboard.putNumber("JESSICA IS SUPER SMART", pivotMotor.getEncoder().getPosition());
    // SmartDashboard.putNumber("Telescoping Current", telescopingMotor.getStatorCurrent().getValue());
    
  }

  public double getPivotEncoder() {
    return absoluteEncoder.getPosition();
  }

  public double getArmEncoder() {
    return telescopingPos;
  }

  // Intake Functions
  public void cubeIntake() {
    ManipulatorConstants.IS_JESSICA_DUMB = true;
    intakeMotor.set(ManipulatorConstants.CUBE_INTAKE_SPEED);
  }

  public void enabelSoftLimit(){
    // telescopingConfig = new TalonFXConfiguration();
    // motorOutputConfigs = new MotorOutputConfigs();

    // motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    
  
    // telescopingMotor.setRotorPosition(0);
    // telescopingLimitSwitchConfigs.ForwardSoftLimitEnable = true;  
    // telescopingLimitSwitchConfigs.ForwardSoftLimitThreshold = 0;
    // telescopingLimitSwitchConfigs.ReverseSoftLimitEnable = true;
    // telescopingLimitSwitchConfigs.ReverseSoftLimitThreshold = -94;

    // telescopingConfig.SoftwareLimitSwitch = telescopingLimitSwitchConfigs;
    // telescopingMotor.getConfigurator().apply(telescopingConfig);
    // telescopingMotor.getConfigurator().apply(motorOutputConfigs);
    
  }

  public void disableSoftLimit(){
    // telescopingConfig = new TalonFXConfiguration();
    // motorOutputConfigs = new MotorOutputConfigs();

    // motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    // telescopingLimitSwitchConfigs.ForwardSoftLimitEnable = false;  
    // telescopingLimitSwitchConfigs.ReverseSoftLimitEnable = false;

    // telescopingConfig.SoftwareLimitSwitch = telescopingLimitSwitchConfigs;
    // telescopingMotor.getConfigurator().apply(telescopingConfig);
    // telescopingMotor.getConfigurator().apply(motorOutputConfigs);
  }
  public void cubeScore() {
    ManipulatorConstants.IS_JESSICA_DUMB = false;
    intakeMotor.set(ManipulatorConstants.CUBE_SCORE_SPEED);
  }

  public void coneScore() {
    intakeMotor.set(ManipulatorConstants.INTAKE_MOTOR_SPEED); // 0.238092005252838
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

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

  public void holdPivot(){
    pivotPIDController.setReference(ManipulatorConstants.PIVOT_RETRACTED_POSITION, CANSparkMax.ControlType.kPosition);
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

  // sensor functions
  // public boolean getConeSensor() {
  //   return coneSensor.get();
  // }

  // public boolean getCubeSensor() {
  //   return cubeSensor.get();
  // }
}