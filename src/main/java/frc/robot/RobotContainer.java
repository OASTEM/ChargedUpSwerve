// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Balance;
import frc.robot.commands.PadDrive;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.StartIntake;
//import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Manipulator.Intake;
import frc.robot.utils.LogitechGamingPad;

import java.util.HashMap;
import java.util.function.Consumer;

import javax.net.ssl.SSLSocket;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.Limelight;
//import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.LogitechGamingPad;
import frc.robot.utils.ShuffleboardComponents;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = Robot.swerveSubsystem;
  private final Intake intake = new Intake();
  private final LogitechGamingPad pad = new LogitechGamingPad(0);
  private final Limelight limelight = new Limelight();
  private final ShuffleboardComponents components = new ShuffleboardComponents();
  // private final NavX navX = new NavX();

  // Buttons
  private final JoystickButton padA = new JoystickButton(pad, 1);
  private final JoystickButton padB = new JoystickButton(pad, 2);
  private final JoystickButton padX = new JoystickButton(pad, 3);
  private final JoystickButton padY = new JoystickButton(pad, 4);
  private final JoystickButton rightBumper = new JoystickButton(pad, 6);
  private final JoystickButton leftBumper = new JoystickButton(pad, 5);

  PathPlannerTrajectory examplePath = PathPlanner.loadPath("Test Path", new PathConstraints(4, 3)); // in m/s
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(
        new PadDrive(
            swerveSubsystem, pad, true, limelight, Constants.SwerveConstants.usingVision));
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    padA.onTrue(new InstantCommand(swerveSubsystem::addRotorPositionsforModules));
    padB.onTrue(new InstantCommand(swerveSubsystem::zeroHeading));
    padY.onTrue(new InstantCommand(swerveSubsystem::configSlowMode));
    padX.whileTrue(new Balance(swerveSubsystem));
    
    leftBumper.whileTrue(new StartIntake(intake));
    rightBumper.whileTrue(new ReverseIntake(intake));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  private void configureAutoCommands(){
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("Test Path", new PathConstraints(4, 3)); // in m/s
  }
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    Consumer<SwerveModuleState[]> outputModuleStates = moduleStates -> {
      // Iterate through the SwerveModuleState array and print each module's state
      for (int i = 0; i < moduleStates.length; i++) {
          SwerveModuleState moduleState = moduleStates[i];
          System.out.println("Module " + i + " - Angle: " + moduleState.angle + ", Speed: " + moduleState.speedMetersPerSecond);
          // Add more print statements or processing as needed
      }
    };
    return swerveSubsystem.followTrajectoryCommand(examplePath, false, outputModuleStates);
  }
}

