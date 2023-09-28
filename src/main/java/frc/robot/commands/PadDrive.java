// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.LogitechGamingPad;

public class PadDrive extends CommandBase {

  private final SwerveSubsystem swerveSubsystem;
  private final boolean isFieldOriented;
  private final LogitechGamingPad pad;
  private final Limelight limelight;
  private boolean vision;
  private double turn;
  private double heading_deadband = 0.2;
  private double controller_deadband = 0.1;

  /** Creates a new SwerveJoystick. */
  public PadDrive(SwerveSubsystem swerveSubsystem,
      LogitechGamingPad pad,
      boolean isFieldOriented, Limelight limelight, boolean vision) {
    this.swerveSubsystem = swerveSubsystem;
    this.pad = pad;
    this.isFieldOriented = isFieldOriented;
    this.limelight = limelight;
    this.vision = vision;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //swerveSubsystem.test(0, pad.getLeftAnalogXAxis() * MotorConstants.MAX_SPEED, pad.getRightAnalogXAxis() * MotorConstants.MAX_ANGULAR_SPEED);
    
    if (vision)
    {
      updatePosition();
    }

    double y;
    double x;

    if (Constants.MotorConstants.SLOW_MODE){
      y = pad.getLeftAnalogXAxis() * MotorConstants.MAX_SPEED * 0.5;
      x = pad.getLeftAnalogYAxis() * -MotorConstants.MAX_SPEED * 0.5;

    }

    else if (Constants.MotorConstants.AACORN_MODE){
      y = pad.getLeftAnalogXAxis() * MotorConstants.MAX_SPEED * 0.85;
      x = pad.getLeftAnalogYAxis() * -MotorConstants.MAX_SPEED * 0.85;

    }

    else{
      y = pad.getLeftAnalogXAxis() * MotorConstants.MAX_SPEED;
      x = pad.getLeftAnalogYAxis() * -MotorConstants.MAX_SPEED;
    }
   

    if (Math.abs(pad.getLeftAnalogXAxis()) < Constants.SwerveConstants.JESSICA){
      y = 0;
    }

    if (Math.abs(pad.getLeftAnalogYAxis()) < Constants.SwerveConstants.JESSICA){
      x = 0;
    }

    // If the right joystick is within the deadband, don't turn
    if (Math.abs(pad.getRightAnalogXAxis()) <= controller_deadband) {
      if (MotorConstants.HEADING > (swerveSubsystem.getHeading() + heading_deadband)) {
        turn = -MotorConstants.MAX_ANGULAR_SPEED * 0.5;
      } else if (MotorConstants.HEADING < (swerveSubsystem.getHeading() - heading_deadband)) {
        turn = MotorConstants.MAX_ANGULAR_SPEED * 0.5;
      } else {
        turn = 0;
      }
    } else {
      turn = pad.getRightAnalogXAxis() * MotorConstants.MAX_ANGULAR_SPEED;
      MotorConstants.HEADING = swerveSubsystem.getHeading();
    }

    if (Constants.MotorConstants.AACORN_MODE){
      swerveSubsystem.drive(x * Constants.MotorConstants.AACORN_SPEED, y * Constants.MotorConstants.AACORN_SPEED, turn * Constants.MotorConstants.TURN_CONSTANT, isFieldOriented);
    }

    else {
      swerveSubsystem.drive(x * Constants.MotorConstants.SPEED_CONSTANT, y * Constants.MotorConstants.SPEED_CONSTANT, turn * Constants.MotorConstants.TURN_CONSTANT, isFieldOriented);
    }
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void updatePosition(){
    swerveSubsystem.addVision(limelight.getRobotPosition());
  }

}
