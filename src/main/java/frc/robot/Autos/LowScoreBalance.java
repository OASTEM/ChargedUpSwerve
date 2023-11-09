// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.BalanceFront;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.ManipulatorCommands.MovePivot;
import frc.robot.commands.ManipulatorCommands.MoveTele;
import frc.robot.commands.ManipulatorCommands.Retract;
import frc.robot.commands.ManipulatorCommands.ScoreCube;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LowScoreBalance extends SequentialCommandGroup {
  /** Creates a new LowScoreBalance. */
  public LowScoreBalance(SwerveSubsystem swerveSubsystem, Manipulator manipulator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(swerveSubsystem::zeroHeading),
      new InstantCommand(swerveSubsystem::addRotorPositionsforModules).withTimeout(0.5),
      new MovePivot(manipulator, ManipulatorConstants.PIVOT_LOW_POSITION).withTimeout(2),
      new MoveTele(manipulator, ManipulatorConstants.TELE_LOW_POSITION).withTimeout(0.2),
      new ScoreCube(manipulator).withTimeout(1),
      new Retract(manipulator).withTimeout(3),
      new DriveStraight(swerveSubsystem, -1, manipulator).withTimeout(3), // wut this
      new BalanceFront(swerveSubsystem),
      new InstantCommand(swerveSubsystem::heading180)
    );
  }
}