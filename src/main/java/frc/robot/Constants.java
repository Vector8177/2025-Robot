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

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
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

  public static final class ElevatorConstants {}

  public static final class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 60;
    public static final int maxIntakeMotorVoltage = 12;
    public static final double INTAKE_SPEED = .8d;
  }

  public static final class WristConstants {
    public static final int wristMotorId = 61;
    public static final double maxMotorVoltage = 10.0;

    public static final double WIRST_INTAKE_POSITION = -1; // Change this later
    public static final double WRIST_SCORING_POSITION = -1; // Change this later

    public enum PIDFFmode {
      WEIGHTED(weightedP, weightedI, weightedD, weightedS, weightedV, weightedA, weightedG),
      UNWEIGHTED(
          unweightedP,
          unweightedI,
          unweightedD,
          unweightedS,
          unweightedV,
          unweightedA,
          unweightedG);

      public final double kP;
      public final double kI;
      public final double kD;
      public final double kS;
      public final double kV;
      public final double kA;
      public final double kG;

      private PIDFFmode(
          double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
      }
    }

    public static double weightedP = 2.8;
    public static double weightedI = 0.0;
    public static double weightedD = 0.2;

    public static double weightedS = 0.274235; // Old Value: 0.4361
    public static double weightedV = 0.67715; // Old Value: 0.79036
    public static double weightedA = 0.020744; // Old Value: 0.0
    public static double weightedG = 0.81416; // Old Value: 0.86416

    public static double unweightedP = 2.0;
    public static double unweightedI = 0.0;
    public static double unweightedD = 0.2;

    public static double unweightedS = 0.11237;
    public static double unweightedV = 0.56387;
    public static double unweightedA = 0.041488;
    public static double unweightedG = 0.76416;

    public static final double motorGearRatio = 1 / 32.0;
    public static final double absoluteEncoderOffset = 5.412927;

    public static final int currentLimit = 40;
  }

  public final class VisionConstants {
    // public static final String bottomCameraName = "limelight-bottom";
    // public static final Transform3d bottomCameraPosition =
    //     new Transform3d(
    //         new Translation3d(
    //             Units.inchesToMeters(9.1505),
    //             Units.inchesToMeters(9.666),
    //             Units.inchesToMeters(31.185)),
    //         new Rotation3d(0, Units.degreesToRadians(20), Units.degreesToRadians(5)));

    // public static final String topCameraName = "limelight-top";
    // public static final Transform3d topCameraPosition =
    //     new Transform3d(
    //         new Translation3d(
    //             Units.inchesToMeters(9.1505),
    //             Units.inchesToMeters(-9.666),
    //             Units.inchesToMeters(31.185)),
    //         new Rotation3d(0, Units.degreesToRadians(20), Units.degreesToRadians(-5)));

    public static final class PoseEstimation {
      public interface StandardDeviation {
        Vector<N3> forMeasurement(double distance, int count);
      }

      /**
       * Standard deviations of model states. Increase these numbers to trust your model's state
       * estimates less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
       * radians.
       */
      public static final StandardDeviation PHOTON_VISION_STD_DEV =
          (distance, count) -> {
            double distanceMultiplier = Math.pow(distance - ((count - 1) * 2), 2);
            double translationalStdDev = (0.05 / (count)) * distanceMultiplier + 0.05;
            double rotationalStdDev = 0.2 * distanceMultiplier + 0.1;
            return VecBuilder.fill(translationalStdDev, translationalStdDev, rotationalStdDev);
          };

      public static final double POSE_AMBIGUITY_CUTOFF = .3;

      public static final double POSE_DISTANCE_CUTOFF = FieldConstants.fieldLength / 2;
    }

    public static class FieldConstants {
      public static final double fieldLength = 16.542;
      public static final double fieldWidth = 8.0137;
    }
  }

  public static final class ClimberConstants {
    public static final int RIGHT_MOTOR_ID = 51;
    public static final int LEFT_MOTOR_ID = 50;

    public static final double CLIMBER_GEAR_RATIO = 1 / 125d; // sus

    public static final double rightClimberAbsoluteEncoderOffset = 0d;
    public static final double leftClimberAbsoluteEncoderOffset = 0d;

    public static final int maxClimberMotorVoltage = 12;

    public static final double climberKP = 1d;
    public static final double climberKI = 0d;
    public static final double climberKD = 0d;

    public static final double climberSpeed = 30d;

    public static final double climberTopLimit = 20000d;
    public static final double climberBottomLimit = 0.2d;
  }
}
