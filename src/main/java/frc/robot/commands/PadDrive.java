// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.LogitechGamingPad;

public class PadDrive extends CommandBase {

  private final SwerveSubsystem swerveSubsystem;
  private final boolean isFieldOriented;
  private final LogitechGamingPad pad;

  /** Creates a new SwerveJoystick. */
  public PadDrive(SwerveSubsystem swerveSubsystem,
      LogitechGamingPad pad,
      boolean isFieldOriented) {
    this.swerveSubsystem = swerveSubsystem;
    this.pad = pad;
    this.isFieldOriented = isFieldOriented;

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

    System.out.println("Inside PadDrive");
    double x = pad.getLeftAnalogXAxis() * MotorConstants.MAX_SPEED;
    double y = pad.getLeftAnalogYAxis() * MotorConstants.MAX_SPEED;
    double turn = pad.getRightAnalogXAxis() * MotorConstants.MAX_ANGULAR_SPEED;
    swerveSubsystem.drive(x, y, turn, isFieldOriented);
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
}
