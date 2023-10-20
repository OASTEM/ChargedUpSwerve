// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManipulatorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.Manipulator;

public class ScoringPosition extends CommandBase {
  /** Creates a new ScoringPosition. */
  private Manipulator manipulator;
  public ScoringPosition(Manipulator manipulator) {
    this.manipulator = manipulator;
    addRequirements(manipulator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (ManipulatorConstants.scoring_pos == 3){
      ManipulatorConstants.scoring_pos = 0;
    }

    else {
      ManipulatorConstants.scoring_pos += 1;
    }


    if (ManipulatorConstants.scoring_pos  == 0) {
      manipulator.setTelescopingPosition(2.5);
    }

    else if (ManipulatorConstants.scoring_pos == 1)
    {
      manipulator.setPivotPosition(ManipulatorConstants.PIVOT_LOW_POSITION);
    }
    else if (ManipulatorConstants.scoring_pos == 2)
    {
      manipulator.setPivotPosition(ManipulatorConstants.PIVOT_MID_POSITION);
    }
    else if (ManipulatorConstants.scoring_pos >= 3)
    {
      manipulator.setPivotPosition(ManipulatorConstants.PIVOT_HIGH_POSITION);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (ManipulatorConstants.scoring_pos  == 0) {
      manipulator.setPivotPosition(0);
    }

    else if (ManipulatorConstants.scoring_pos == 1)
    {
      manipulator.setTelescopingPosition(ManipulatorConstants.TELE_LOW_POSITION);
    }
    else if (ManipulatorConstants.scoring_pos == 2)
    {
      manipulator.setTelescopingPosition(ManipulatorConstants.TELE_MID_POSITION);
    }
    else if (ManipulatorConstants.scoring_pos >= 3)
    {
      manipulator.setTelescopingPosition(ManipulatorConstants.TELE_HIGH_POSITION);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ended*****************************************");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
