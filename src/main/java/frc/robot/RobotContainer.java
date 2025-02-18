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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.MainCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Climber climber;
  private final Elevator elevator;
  private final Wrist wrist;
  private final Vision vision;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        intake = new Intake(new IntakeIOTalonFX()); // rename string to canbussname

        climber = new Climber(new ClimberIOTalonFX());

        elevator = new Elevator(new ElevatorIOTalonFX());

        wrist = new Wrist(new WristIOTalonFX());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight("limelight-bottom", drive::getRotation),
                new VisionIOLimelight("limelight-top", drive::getRotation));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        intake = new Intake(new IntakeIOTalonFX()); // rename string to canbussname

        climber = new Climber(new ClimberIOTalonFX());

        elevator = new Elevator(new ElevatorIOTalonFX());

        wrist = new Wrist(new WristIOTalonFX());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight("limelight-bottom", drive::getRotation),
                new VisionIOLimelight("limelight-top", drive::getRotation));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        intake = new Intake(new IntakeIOTalonFX()); // rename string to canbussname

        climber = new Climber(new ClimberIOTalonFX());

        elevator = new Elevator(new ElevatorIOTalonFX());

        wrist = new Wrist(new WristIOTalonFX());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight("limelight-bottom", drive::getRotation),
                new VisionIOLimelight("limelight-top", drive::getRotation));
        break;
    }

    NamedCommands.registerCommand("Stop intake", MainCommands.stopIntake(intake));
    NamedCommands.registerCommand("Run intake", MainCommands.runIntake(intake));
    NamedCommands.registerCommand("Run outtake", MainCommands.runOuttake(intake));

    NamedCommands.registerCommand(
        "Set wrist intake position", MainCommands.setWristIntakePosition(wrist));
    NamedCommands.registerCommand(
        "Set wrist scoring position", MainCommands.setWristScoringPosition(wrist));

    NamedCommands.registerCommand("Set climber up", MainCommands.setClimberUp(climber));
    NamedCommands.registerCommand("Set climber down", MainCommands.setClimberDown(climber));
    NamedCommands.registerCommand("Stop climber", MainCommands.stopClimber(climber));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));
    driverController
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    PIDController aimController = new PIDController(0.1, 0.0, 0.0);
    aimController.enableContinuousInput(-Math.PI, Math.PI);

    final double alignSpeed = .4;
    final double alignRange = 2.75;
    driverController
        .a()
        .whileTrue(
            Commands.sequence(
                DriveCommands.joystickDrive(
                    drive,
                    () ->
                        LimelightHelpers.getTX("limelight-bottom")
                                > alignRange * Math.cos(drive.getRotation().getRadians())
                            ? -alignSpeed * Math.cos(drive.getRotation().getRadians())
                            : LimelightHelpers.getTX("limelight-bottom")
                                    < -alignRange * Math.cos(drive.getRotation().getRadians())
                                ? alignSpeed * Math.cos(drive.getRotation().getRadians())
                                : 0,
                    () ->
                        LimelightHelpers.getTX("limelight-bottom")
                                > alignRange * Math.sin(drive.getRotation().getRadians())
                            ? -alignSpeed * Math.sin(drive.getRotation().getRadians())
                            : LimelightHelpers.getTX("limelight-bottom")
                                    < -alignRange * Math.sin(drive.getRotation().getRadians())
                                ? alignSpeed * Math.sin(drive.getRotation().getRadians())
                                : 0,
                    () -> 0)));

    driverController
        .b()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () ->
                    LimelightHelpers.getTXNC("limelight-bottom") < 4
                            && LimelightHelpers.getTXNC("limelight-bottom") > -4
                        ? 0
                        : LimelightHelpers.getTXNC("limelight-bottom") > 4 ? -.5 : .5));

    // operatorController.a().onTrue(MainCommands.setElevatorPosition(elevator, 20));
    operatorController.povUp().whileTrue(MainCommands.setElevatorVoltage(elevator, 2));
    operatorController.povDown().whileTrue(MainCommands.setElevatorVoltage(elevator, -2));
    operatorController
        .rightTrigger()
        .onTrue(MainCommands.runIntake(intake))
        .onFalse(MainCommands.stopIntake(intake));
    operatorController
        .leftTrigger()
        .onTrue(MainCommands.runOuttake(intake))
        .onFalse(MainCommands.stopIntake(intake));
    operatorController.povLeft().onTrue(MainCommands.setWristIntakePosition(wrist));
    operatorController.povRight().onTrue(MainCommands.setWristScoringPosition(wrist));
    operatorController
        .rightBumper()
        .onTrue(MainCommands.setClimberUp(climber))
        .onFalse(MainCommands.stopClimber(climber));
    operatorController
        .leftBumper()
        .onTrue(MainCommands.setClimberDown(climber))
        .onFalse(MainCommands.stopClimber(climber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
