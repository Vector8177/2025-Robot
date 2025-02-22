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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class ElevatorConstants {
    public static final int LEFT_ELEVATOR_MOTOR_ID = 31; // lead motor
    public static final int RIGHT_ELEVATOR_MOTOR_ID = 32;

    public static final int ELEVATOR_INTAKE = 2; // change this later
    public static final int ELEVATOR_L1 = 5; // change this later
    public static final int ELEVATOR_L2 = 15; // change this later
    public static final int ELEVATOR_L3 = 20; // change this later
    public static final int ELEVATOR_L4 = 25; // change this later
    public static final int ELEVATOR_NET = 45; // change this later

    public static final int MAX_ELEVATOR_VOLTAGE = 12;

    public static double kP = 2.5;
    public static double kI = 0.0;
    public static double kD = 0.1;

    public static double kS = 0.11237;
    public static double kV = 0.56387;
    public static double kA = 0.041488;
    public static double kG = 0.76416;
  }

  public static final class IntakeConstants {

    public static final int INTAKE_MOTOR_ID = 42;
    public static final int MAX_INTAKE_VOLTAGE = 12;
    public static final double INTAKE_SPEED = 1d;
  }

  public static final class WristConstants {
    public static final int WRIST_MOTOR_ID = 41;
    public static final int MAX_WRIST_VOLTAGE = 12;

    public static final double WIRST_INTAKE_POSITION = 5; // Change this later
    public static final double WRIST_SCORING_POSITION_L1 = 1.5; // Change this later
    public static final double WRIST_SCORING_POSITION_L2 = 2; // Change this later; L2 and L3 should be the same
    public static final double WRIST_SCORING_POSITION_L4 = 3; // Change this later
    public static final double WRIST_PERPENDICULAR_POSITION = 3; // Change this later
    public static final double WRIST_SCORING_POSITION_NET = 3; // Change this later

    public static double kP = 0.5;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double kS = 0.11237;
    public static double kV = 0.56387;
    public static double kA = 0.041488;
    public static double kG = 0.76416;
  }

  public final class VisionConstants {
    public static final double alignSpeed = .4;
    public static final double alignRange = 3;
    public static final double closeAlignSpeed = .25;
    public static final double closeAlignRange = 1;

    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "limelight-bottom";
    public static String camera1Name = "limelight-top";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 =
        new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
    public static Transform3d robotToCamera1 =
        new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
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
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor =
        Double.POSITIVE_INFINITY; // No rotation data available
  }

  public static final class ClimberConstants {
    public static final int RIGHT_MOTOR_ID = 52;
    public static final int LEFT_MOTOR_ID = 51;

    public static final int maxClimberMotorVoltage = 12;
  }
}
