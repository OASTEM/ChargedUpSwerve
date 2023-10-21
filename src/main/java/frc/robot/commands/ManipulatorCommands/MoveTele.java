// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManipulatorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;

import frc.robot.Constants.ManipulatorConstants;

public class MoveTele extends CommandBase {
  /** Creates a new RetractArm. */
  private Manipulator manipulator;
  private double value;

  public MoveTele(Manipulator manipulator, double value) {
    // Use addRequirements() here to d0eclare subsystem dependencies.
    addRequirements(manipulator);
    this.manipulator = manipulator;
    this.value = value;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    manipulator.setTelescopingPosition(value);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(manipulator.getArmEncoder()) - value < 0.013){
      return true;
    }
    return false;
  }
}
