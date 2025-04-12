package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

// Positions Robot at the Nearest Valid Target
public class AutoAlign extends Command {

  // Instantiate Stuff
  Drive m_swerveSubsystem;

  // Timer for cancellation with failed robot adjustment
  Timer timer = new Timer();

  PIDController m_aimController =
      new PIDController(VisionConstants.kP_aim, VisionConstants.kI_aim, VisionConstants.kD_aim);
  PIDController m_rangeController =
      new PIDController(
          VisionConstants.kP_range, VisionConstants.kI_range, VisionConstants.kD_range);
  PIDController m_strafeController =
      new PIDController(
          VisionConstants.kP_strafe, VisionConstants.kI_strafe, VisionConstants.kD_strafe);

  // Bot Pose Target Space Relative [TX, TY, TZ, Pitch, Yaw, Roll]
  private double[] botPoseTargetSpace = new double[6];
  private double[] botPoseTargetSpace2 = new double[6];

  private final boolean m_isReefRight;

  /*
   * Tag Guide (Perspective is from respective DS):
   * 1: Coral Station Red Left
   * 2: Coral Station Red Right
   * 3: Processor Blue
   * 4: Barge Blue Back
   * 5: Barge Red Front
   * 6: Reef Red Front Left
   * 7: Reef Red Front Center
   * 8: Reef Red Front Right
   * 9: Reef Red Back Right
   * 10: Reef Red Back Center
   * 11: Reef Red Back Left
   * 12: Coral Station Blue Right
   * 13: Coral Station Blue Left
   * 14: Barge Blue Front
   * 15: Barge Red Back
   * 16: Processor Red
   * 17: Reef Blue Front Right
   * 18: Reef Blue Front Center
   * 19: Reef Blue Front Left
   * 20: Reef Blue Back Left
   * 21: Reef Blue Back Center
   * 22: Reef Blue Back Right
   */

  // All the Valid IDs available for positioning
  int[] validIDs = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};

  // Lil boolean for checking for "Tag In View"
  private boolean tiv;
  private boolean tiv2;

  // Constants
  private double m_rangeTarget; // forward
  private double m_strafeTarget; // sideways
  private double m_aimTarget; // rotation

  // Constructor
  public AutoAlign(Drive driveSubsystem, boolean isReefRight) {

    // Definitions and setting parameters are equal to members!
    m_swerveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);

    m_isReefRight = isReefRight;
  }

  // What we do to set up the command
  public void initialize() {

    // Reset the Shoot Commit Boolean
    VisionConstants.k_positioned = true;

    // Adds condition that filters out undesired IDs
    LimelightHelpers.SetFiducialIDFiltersOverride(VisionConstants.camera0Name, validIDs);

    // Update BotPoseTargetSpace 2 is using left camera
    botPoseTargetSpace =
        NetworkTableInstance.getDefault()
            .getTable(VisionConstants.camera0Name)
            .getEntry("botpose_targetspace")
            .getDoubleArray(new double[6]);

    botPoseTargetSpace2 =
        NetworkTableInstance.getDefault()
            .getTable(VisionConstants.camera1Name)
            .getEntry("botpose_targetspace")
            .getDoubleArray(new double[6]);

    // Checks for TIV
    tiv =
        (LimelightHelpers.getTV(VisionConstants.camera0Name)
            && botPoseTargetSpace[2] > VisionConstants.k_tzValidRange
            && Math.abs(botPoseTargetSpace[4]) < VisionConstants.k_yawValidRange);
    tiv2 =
        (LimelightHelpers.getTV(VisionConstants.camera1Name)
            && botPoseTargetSpace2[2] > VisionConstants.k_tzValidRange
            && Math.abs(botPoseTargetSpace2[4]) < VisionConstants.k_yawValidRange);

    // Set Constants
    if (m_isReefRight) {
      m_strafeTarget = VisionConstants.k_strafeReefRightTarget;
      m_rangeTarget = VisionConstants.k_rangeReefRightTarget;
      m_aimTarget = VisionConstants.k_aimReefRightTarget;
    } else {
      m_strafeTarget = VisionConstants.k_strafeReefLeftTarget;
      m_rangeTarget = VisionConstants.k_rangeReefLeftTarget;
      m_aimTarget = VisionConstants.k_aimReefLeftTarget;
    }

    // Timer Reset
    timer.start();
    timer.reset();
  }

  // The actual control!
  public void execute() {

    VisionConstants.k_positioning = true;

    // Update the pose from NetworkTables (Limelight Readings)
    botPoseTargetSpace =
        NetworkTableInstance.getDefault()
            .getTable(VisionConstants.camera0Name)
            .getEntry("botpose_targetspace")
            .getDoubleArray(new double[6]);

    botPoseTargetSpace2 =
        NetworkTableInstance.getDefault()
            .getTable(VisionConstants.camera1Name)
            .getEntry("botpose_targetspace")
            .getDoubleArray(new double[6]);

    if (timer.get() > 1.5 || !tiv) VisionConstants.k_positioned = false;

    // Checks for a continued valid pose
    if (m_isReefRight == false && tiv) {
      tiv =
          LimelightHelpers.getTV(VisionConstants.camera0Name)
              && botPoseTargetSpace[2] > VisionConstants.k_tzValidRange;
      m_swerveSubsystem.runVelocity(
          new ChassisSpeeds(limelight_range_PID(), limelight_strafe_PID(), limelight_aim_PID()));
    } else if (m_isReefRight == true && tiv2) {
      tiv2 =
          LimelightHelpers.getTV(VisionConstants.camera1Name)
              && botPoseTargetSpace2[2] > VisionConstants.k_tzValidRange;
      m_swerveSubsystem.runVelocity(
          new ChassisSpeeds(limelight_range_PID(), limelight_strafe_PID(), limelight_aim_PID()));
    }
  }

  // Add stuff we do after to reset here (a.k.a tag filters)
  public void end(boolean interrupted) {
    VisionConstants.k_positioning = false;
  }

  // Are we done yet? Finishes when threshold is reached or if no tag in view or if timer is reached
  public boolean isFinished() {
    if (!m_isReefRight) {
      return (
              // Strafe (Right Right Positioning)
              Math.abs(botPoseTargetSpace[0] - m_strafeTarget) < VisionConstants.k_strafeThreshold)
              &&
              // Range (Distance to Tag)
              Math.abs(botPoseTargetSpace[2] - m_rangeTarget) < VisionConstants.k_rangeThreshold
              &&
              // Aim (Angle)
              Math.abs(botPoseTargetSpace[4] - m_aimTarget) < VisionConstants.k_aimThreshold

          // Other quit conditions
          || !tiv
          || timer.get() > 1.5;
    } else {
      return (
              // Strafe (Right Right Positioning)
              Math.abs(botPoseTargetSpace2[0] - m_strafeTarget) < VisionConstants.k_strafeThreshold)
              &&
              // Range (Distance to Tag)
              Math.abs(botPoseTargetSpace2[2] - m_rangeTarget) < VisionConstants.k_rangeThreshold
              &&
              // Aim (Angle)
              Math.abs(botPoseTargetSpace2[4] - m_aimTarget) < VisionConstants.k_aimThreshold

          // Other quit conditions
          || !tiv2
          || timer.get() > 1.5;
    }
  }

  // Advanced PID-assisted ranging control with Limelight's TX value from target-relative data
  private double limelight_strafe_PID() {

    // Limelight X Axis Range in Meters
    m_strafeController.enableContinuousInput(-2, 2);

    // Calculates response based on difference in horizontal distance from tag to robot
    double targetingStrafeSpeed;
    if (!m_isReefRight) {
      targetingStrafeSpeed = m_strafeController.calculate(botPoseTargetSpace[0] - m_strafeTarget);
    } else {
      targetingStrafeSpeed = m_strafeController.calculate(botPoseTargetSpace2[0] - m_strafeTarget);
    }

    // Value scale up to robot max speed (Double can't exceed 1.0)
    targetingStrafeSpeed *= -1.0 * m_swerveSubsystem.getMaxLinearSpeedMetersPerSec();
    Logger.recordOutput("Sideways PID Speed", targetingStrafeSpeed);

    return targetingStrafeSpeed;
  }

  // Advanced PID-assisted ranging control with Limelight's TZ value from target-relative data
  private double limelight_range_PID() {

    // Limelight Z Axis Range in meters
    m_rangeController.enableContinuousInput(-2, 0);

    // Calculates response based on difference in distance from tag to robot
    double targetingForwardSpeed;
    if (!m_isReefRight) {
      targetingForwardSpeed = m_rangeController.calculate(botPoseTargetSpace[2] - m_rangeTarget);
    } else {
      targetingForwardSpeed = m_rangeController.calculate(botPoseTargetSpace2[2] - m_rangeTarget);
    }

    // Value scale up to robot max speed and invert (double cannot exceed 1.0)
    targetingForwardSpeed *= m_swerveSubsystem.getMaxLinearSpeedMetersPerSec();
    Logger.recordOutput("Forward PID Speed", targetingForwardSpeed);
    // records PID speed for further us as a variable
    return targetingForwardSpeed;
  }

  // Advanced PID-assisted ranging control with Limelight's Yaw value from target-relative data
  private double limelight_aim_PID() {

    // Limelight Yaw Angle in Degrees
    m_aimController.enableContinuousInput(-30, 30);

    // Calculates response based on difference in angle from tag to robot
    double targetingAngularVelocity;
    if (!m_isReefRight) {
      targetingAngularVelocity = m_aimController.calculate(botPoseTargetSpace[4] - m_aimTarget);
    } else {
      targetingAngularVelocity = m_aimController.calculate(botPoseTargetSpace2[4] - m_aimTarget);
    }

    // Multiply by -1 because robot is CCW Positive. Multiply by a reduction
    // multiplier to reduce speed. Scale TX up with robot speed.
    targetingAngularVelocity *= -0.1 * m_swerveSubsystem.getMaxAngularSpeedRadPerSec();
    Logger.recordOutput("Angular PID Speed", targetingAngularVelocity);

    return targetingAngularVelocity;
  }
}
