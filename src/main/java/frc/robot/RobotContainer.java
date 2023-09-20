// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.BalanceFront;
import frc.robot.commands.PadDrive;
import frc.robot.commands.ManipulatorCommands.ExtendHigh;
import frc.robot.commands.ManipulatorCommands.ExtendLow;
import frc.robot.commands.ManipulatorCommands.ExtendMid;
import frc.robot.commands.ManipulatorCommands.IntakeFeeder;
import frc.robot.commands.ManipulatorCommands.IntakeGround;
import frc.robot.commands.ManipulatorCommands.ScoreCone;
import frc.robot.commands.ManipulatorCommands.ScoreCube;
import frc.robot.subsystems.SwerveSubsystem;
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
import frc.robot.subsystems.Manipulator;
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
  private final SwerveSubsystem swerveSubsystem;
  private final Manipulator manipulator;
  private final LogitechGamingPad pad;
  private final LogitechGamingPad opPad;
  private final Limelight limelight;
  private final ShuffleboardComponents components;
  
  private final JoystickButton XboxPadA;
  private final JoystickButton XboxPadB;
  private final JoystickButton XboxPadX;
  private final JoystickButton XboxPadY;
  private final JoystickButton XboxRightBumper;
  private final JoystickButton XboxLeftBumper;

  private final JoystickButton PadA;
  private final JoystickButton PadB;
  private final JoystickButton PadX;
  private final JoystickButton PadY;
  private final JoystickButton RightBumper;
  private final JoystickButton LeftBumper;
  private final JoystickButton StartButton;
  
  private final PathPlannerTrajectory examplePath;
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveSubsystem = Robot.swerveSubsystem;
    manipulator = new Manipulator();
    pad = new LogitechGamingPad(0);
    opPad = new LogitechGamingPad(1);
    limelight = new Limelight();
    components = new ShuffleboardComponents();
    
    XboxPadA = new JoystickButton(pad, 1);
    XboxPadB = new JoystickButton(pad, 2);
    XboxPadX = new JoystickButton(pad, 3);
    XboxPadY = new JoystickButton(pad, 4);
    XboxRightBumper = new JoystickButton(pad, 6);
    XboxLeftBumper = new JoystickButton(pad, 5);

    PadA = new JoystickButton(opPad, 1);
    PadB = new JoystickButton(opPad, 2);
    PadX = new JoystickButton(opPad, 3);
    PadY = new JoystickButton(opPad, 4);
    RightBumper = new JoystickButton(opPad, 6);
    LeftBumper = new JoystickButton(opPad, 5);
    StartButton = new JoystickButton(opPad, 8);
    
    examplePath = PathPlanner.loadPath("Test Path", new PathConstraints(4, 3)); // in m/s
    
    swerveSubsystem.setDefaultCommand(
        new PadDrive(
            swerveSubsystem, pad, true, limelight, Constants.SwerveConstants.usingVision));
    
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
    //Drive Commands
    XboxPadA.onTrue(new InstantCommand(swerveSubsystem::addRotorPositionsforModules));
    XboxPadB.onTrue(new InstantCommand(swerveSubsystem::zeroHeading));
    XboxPadY.onTrue(new InstantCommand(swerveSubsystem::configSlowMode));
    XboxPadX.whileTrue(new BalanceFront(swerveSubsystem));

    //Manipulator Commands
    RightBumper.whileTrue(new IntakeGround(manipulator));
    LeftBumper.whileTrue(new ScoreCube(manipulator));
    PadA.onTrue(new ExtendLow(manipulator));
    PadB.onTrue(new ExtendMid(manipulator));
    PadY.onTrue(new ExtendHigh(manipulator));
    PadX.whileTrue(new IntakeFeeder(manipulator));
    StartButton.whileTrue(new ScoreCone(manipulator));

    
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return swerveSubsystem.followTrajectoryCommand(examplePath, true);
  }
}


