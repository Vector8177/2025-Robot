// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class ElevatorConstants {
    public static final int LEFT_MOTOR_ID = 31; // lead motor
    public static final int RIGHT_MOTOR_ID = 32;

    public static final double INTAKE = 15; // 15
    public static final double L1 = 0;
    public static final double L2 = 29; // 29
    public static final double L3 = 60.5; // 62.5
    public static final double L4 = 103; // 101
    public static final double NET = 103.5; // 101

    public static final int MAX_VOLTAGE = 12;

    public static double kP = 2.5;
    public static double kI = 0.0;
    public static double kD = 0.1;

    public static double kS = 0.11237;
    public static double kV = 0.56387;
    public static double kA = 0.041488;
    public static double kG = 0.76416;
  }

  public static final class IntakeConstants {
    public static final int MOTOR_ID = 42;
    public static final int MAX_VOLTAGE = 12;
  }

  public static final class WristConstants {
    public static final int MOTOR_ID = 41;
    public static final int MAX_VOLTAGE = 12;

    // These are relative encoder positions - comments are for absolute encoder -7.87
    public static final double INTAKE_POSITION = -8.6; // .67
    public static final double SCORING_POSITION_L1 = -4.5; // .35
    public static final double L1_AUTO = -5.5; // .35
    public static final double SCORING_POSITION_L2 = -4.75; // .53
    public static final double SCORING_POSITION_L4 = -6.5; // .58
    public static final double PERPENDICULAR_POSITION = -5.7; // .6
    public static final double SCORING_POSITION_NET = -7.8; // .65
    public static final double FLICK_WRIST_POSITION = -7.5; // .525

    public static double kP = 1.5;
    public static double kI = 0.0;
    public static double kD = 0;

    public static double kS = 0.11237;
    public static double kV = 0.56387;
    public static double kA = 0.041488;
    public static double kG = 0.76416;
  }

  public static final class VisionConstants {
    public static final double alignSpeed = -.5;
    public static final double alignRange = 5;
    public static final double closeAlignSpeed = -.25;
    public static final double closeAlignRange = 1;
    public static final double kP = .04;

    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "limelight-bottom";
    public static String camera1Name = "limelight-top";

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1-meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
        new double[] {
          1.0, // Camera 0
          1.0 // Camera 1
        };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = .5; // More stable than full 3D solve .5
    public static double angularStdDevMegatag2Factor =
        Double.POSITIVE_INFINITY; // No rotation data available

    public static double maxAvgTagDistance = 3.0; // change this later

    // Boolean for Left/Right Reef
    public static boolean k_isRightReef = true;

    // Boolean for Committing to Shoot
    public static boolean k_positioned = true;

    // PID for Tag Relative Control for Scoring
    public static final double kP_aim = 0.10;
    public static final double kI_aim = 0.0;
    public static final double kD_aim = 0.0;

    public static final double kP_range = 0.35;
    public static final double kI_range = 0.0;
    public static final double kD_range = 0.0;

    public static final double kP_strafe = 0.35;
    public static final double kI_strafe = 0.0;
    public static final double kD_strafe = 0.0;

    // AimNRange Reef Right
    public static final double k_aimReefRightTarget = 0;
    public static final double k_rangeReefRightTarget = -0.56; //change this later
    public static final double k_strafeReefRightTarget = 0.18;

    // AimNRange Reef Left
    public static final double k_aimReefLeftTarget = 0;
    public static final double k_rangeReefLeftTarget = -0.54; //change this later
    public static final double k_strafeReefLeftTarget = -0.18;

    // Prerequisites
    public static final double k_tzValidRange = -1.5;
    public static final double k_yawValidRange = 35;

    // Thresholds
    public static final double k_rangeThreshold = 0.02;
    public static final double k_strafeThreshold = 0.02;
    public static final double k_aimThreshold = 0.5;

    // For testing
    public static boolean k_positioning = false;

    public static double[] k_botPoseTargetSpace = new double[6];
  }

  public static final class ClimberConstants {
    public static final int RIGHT_MOTOR_ID = 52;
    public static final int LEFT_MOTOR_ID = 51;

    public static final int MAX_VOLTAGE = 12;
  }
}
