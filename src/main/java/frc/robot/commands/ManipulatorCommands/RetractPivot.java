// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManipulatorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;

import frc.robot.Constants.ManipulatorConstants;

public class RetractPivot extends CommandBase {
  /** Creates a new RetractArm. */
  private Manipulator manipulator;


  public RetractPivot(Manipulator manipulator) {
    // Use addRequirements() here to d0eclare subsystem dependencies.
    addRequirements(manipulator);
    this.manipulator = manipulator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    manipulator.setPivotPosition(0.00);
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

    // if (Math.abs(manipulator.getPivotEncoder()) < 0.02){
    //   return true;
    // }
    return false;
  }
}
