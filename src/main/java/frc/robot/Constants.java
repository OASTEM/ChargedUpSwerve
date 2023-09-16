// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.utils.PID;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  private static final String UTILITY_CLASS = "Utility class";

  private Constants() {
    throw new IllegalStateException(UTILITY_CLASS);
  }

  public static class MotorConstants {
    private MotorConstants() {
      throw new IllegalStateException(UTILITY_CLASS);
    }

    public static final int FRONT_LEFT_STEER_ID = 1;
    public static final int FRONT_LEFT_DRIVE_ID = 2;
    public static final int FRONT_RIGHT_STEER_ID = 3;
    public static final int FRONT_RIGHT_DRIVE_ID = 4;
    public static final int BACK_LEFT_STEER_ID = 5;
    public static final int BACK_LEFT_DRIVE_ID = 6;
    public static final int BACK_RIGHT_STEER_ID = 7;
    public static final int BACK_RIGHT_DRIVE_ID = 8;
    public static final int FRONT_LEFT_CAN_CODER_ID = 9;
    public static final int FRONT_RIGHT_CAN_CODER_ID = 10;
    public static final int BACK_LEFT_CAN_CODER_ID = 11;
    public static final int BACK_RIGHT_CAN_CODER_ID = 12;

    public static final double MAX_SPEED = 4.96824;  
    public static final double MAX_ANGULAR_SPEED = 4.24547626559;
    public static final double ENCODER_COUNTS_PER_ROTATION = 1; //2048 for v5, 1 for v6 (rotations)
    public static final double STEER_MOTOR_GEAR_RATIO = 150.0/7; //24
    public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
    public static final double WHEEL_DIAMETER = 0.1;
    public static final double SPEED_CONSTANT = 0.4; //0.3
    public static final double TURN_CONSTANT = 0.5; //0.3
    public static boolean SLOW_MODE = false;
  }

  public static class SwerveConstants {
    private SwerveConstants() {
      throw new IllegalStateException(UTILITY_CLASS);
    }

    public static final Translation2d frontLeftLocation = new Translation2d(0.24, -0.24);
    public static final Translation2d frontRightLocation = new Translation2d(0.24, 0.24);
    public static final Translation2d backLeftLocation = new Translation2d(-0.24, 0.24);
    public static final Translation2d backRightLocation = new Translation2d(-0.24, -0.24);
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        frontLeftLocation,
        frontRightLocation,
        backLeftLocation,
        backRightLocation);
    public static final Pose2d 
    STARTING_POSE = new Pose2d(0, 0, new Rotation2d());
    public static final double STATE_SPEED_THRESHOLD = 0.05;

    public static final double CANCoderValue9 = 0.539794 - 0.5;
    public static final double CANCoderValue10 = 0.984863; 
    public static final double CANCoderValue11 = 0.9174805;
    public static final double CANCoderValue12 = 0.328613 + 0.5;
    public static final double JESSICA = 0.05;
    public static boolean usingVision = false;
  }

  public static class AutoConstants{
    public static final double AUTO_MAX_SPEED = 1;
    public static final double AUTP_MAX_ACCELERATION = 1;
  }

  public static class ManipulatorConstants {
    public static final int INTAKE_MOTOR_ID = 13;
    public static final double intakeSpeed = 1.0;
    public static final double reverseIntakeSpeed = -1.0;

    public static final int PIVOT_MOTOR_ID = 14;
    public static final PID upPID = new PID(0.05, 0.0 , 0.0, 0.0);
    public static final PID downPID = new PID(0.05, 0.0 , 0.0, 0.0);
    public static final PID sidePID = new PID(0.05, 0.0 , 0.0, 0.0);
    public static final PID pivotPID = new PID(0.05, 0.0 , 0.0, 0.0);
 
    public static final int TELESCOPING_MOTOR_ID = 15;
  }

  public static class Balance{
    public static final double P = 0.015;
    public static final double I = 0.00015;
    public static final double D = 0.0008;
  }

  public static class BalanceDebug{
    public static double P;
    public static double I;
    public static double D;
  }

  public static class DebugMode{
    public static boolean debugMode = false;
  }
}
