// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveModule;

public class AutoTest extends CommandBase {
  /** Creates a new AutoTest. */

  private SwerveSubsystem subsystem;
  private PathPlannerTrajectory trajectory;
  private boolean initialPath;
  private boolean useAllianceColor;
  private PPSwerveControllerCommand command;

 public AutoTest(SwerveSubsystem subsystem) {
    this.subsystem = subsystem;
 }


  // Add any additional methods or fields as needed

  private boolean isTrajectoryComplete() {
    // Implement logic to check if the trajectory is complete
    // Return true when the trajectory is finished, otherwise false
    // You can use information from your subsystem or trajectory to determine this.
    
    return false; // Replace with your logic
  }

  // Called when the command is initially scheduled.




  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
