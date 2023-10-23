// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveStraight extends CommandBase {
  /** Creates a new DriveStraight. */
  private SwerveSubsystem swerveSubsystem;
  private double speed;
  private Manipulator manipulator;

  public DriveStraight(SwerveSubsystem swerveSubsystem, double speed, Manipulator manipulator) {
    this.swerveSubsystem = swerveSubsystem;
    this.speed = speed;
    this.manipulator = manipulator;
    addRequirements(swerveSubsystem, manipulator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSubsystem.zeroHeading();
    swerveSubsystem.addRotorPositionsforModules();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveSubsystem.driveStraight(speed);
    manipulator.holdPivot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
