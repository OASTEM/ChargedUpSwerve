// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class Drive extends CommandBase {

  private final SwerveSubsystem swerveSubsystem;
  private final double x, y, turn;
  private final boolean isFieldOriented;

  /** Creates a new SwerveJoystick. */
  public Drive(SwerveSubsystem swerveSubsystem,
      double x, double y,
      double turn,
      boolean isFieldOriented) {

    this.swerveSubsystem = swerveSubsystem;
    this.x = x;
    this.y = y;
    this.turn = turn;
    this.isFieldOriented = isFieldOriented;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
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
