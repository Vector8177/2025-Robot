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

// import com.pathplanner.lib.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.AutoAlign;
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
      case REAL -> {
        // Real robot, instantiate hardware IO implementations
        intake = new Intake(new IntakeIOTalonFX());

        climber = new Climber(new ClimberIOTalonFX());

        elevator = new Elevator(new ElevatorIOTalonFX());

        wrist = new Wrist(new WristIOTalonFX());

        NamedCommands.registerCommand("Stop Intake", MainCommands.stopIntake(intake));
        NamedCommands.registerCommand("Run Intake", MainCommands.runIntake(intake));
        NamedCommands.registerCommand("Run Outake", MainCommands.runOuttake(intake));

        NamedCommands.registerCommand(
            "Set Intake Position", MainCommands.setIntakePosition(wrist, elevator));

        NamedCommands.registerCommand(
            "Set Elevator L1",
            MainCommands.setElevatorPosition(
                wrist, elevator, ElevatorConstants.L1, WristConstants.SCORING_POSITION_L1));
        NamedCommands.registerCommand(
            "Set Elevator L2",
            MainCommands.setElevatorPosition(
                wrist, elevator, ElevatorConstants.L2, WristConstants.SCORING_POSITION_L2));
        NamedCommands.registerCommand(
            "Set Elevator L3",
            MainCommands.setElevatorPosition(
                wrist, elevator, ElevatorConstants.L3, WristConstants.SCORING_POSITION_L2));
        NamedCommands.registerCommand(
            "Set Elevator L4",
            MainCommands.setElevatorPosition(
                wrist, elevator, ElevatorConstants.L4, WristConstants.SCORING_POSITION_L4));

        NamedCommands.registerCommand("Flick Wrist", MainCommands.flickWrist(intake, wrist));

        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        NamedCommands.registerCommand("Auto Align Left", new AutoAlign(drive, false));
        NamedCommands.registerCommand("Auto Align Right", new AutoAlign(drive, true));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));
      }

      case SIM -> {
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        intake = new Intake(new IntakeIOTalonFX());

        climber = new Climber(new ClimberIOTalonFX());

        elevator = new Elevator(new ElevatorIOTalonFX());

        wrist = new Wrist(new WristIOTalonFX());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation));
      }

      default -> {
        intake = new Intake(new IntakeIOTalonFX());

        climber = new Climber(new ClimberIOTalonFX());

        elevator = new Elevator(new ElevatorIOTalonFX());

        wrist = new Wrist(new WristIOTalonFX());

        NamedCommands.registerCommand("Stop Intake", MainCommands.stopIntake(intake));
        NamedCommands.registerCommand("Run Intake", MainCommands.runIntake(intake));
        NamedCommands.registerCommand("Run Outake", MainCommands.runOuttake(intake));

        NamedCommands.registerCommand(
            "Set Intake Position", MainCommands.setIntakePosition(wrist, elevator));

        NamedCommands.registerCommand(
            "Set Elevator L1",
            MainCommands.setElevatorPosition(
                wrist, elevator, ElevatorConstants.L1, WristConstants.SCORING_POSITION_L1));
        NamedCommands.registerCommand(
            "Set Elevator L2",
            MainCommands.setElevatorPosition(
                wrist, elevator, ElevatorConstants.L2, WristConstants.SCORING_POSITION_L2));
        NamedCommands.registerCommand(
            "Set Elevator L3",
            MainCommands.setElevatorPosition(
                wrist, elevator, ElevatorConstants.L3, WristConstants.SCORING_POSITION_L2));
        NamedCommands.registerCommand(
            "Set Elevator L4",
            MainCommands.setElevatorPosition(
                wrist, elevator, ElevatorConstants.L4, WristConstants.SCORING_POSITION_L4));
        NamedCommands.registerCommand("Flick Wrist", MainCommands.flickWrist(intake, wrist));

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));
      }
    }

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
    driverController
        .x()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driverController.getLeftY() * .5,
                () -> -driverController.getLeftX() * .5,
                () -> -driverController.getRightX() * .5));
    driverController.povUp().onTrue(MainCommands.changeElevatorSetpoint(elevator, 1));
    driverController.povDown().onTrue(MainCommands.changeElevatorSetpoint(elevator, -1));
    driverController.povRight().onTrue(new AutoAlign(drive, true));
    driverController.povLeft().onTrue(new AutoAlign(drive, false));
    driverController.b().onTrue(MainCommands.flickWrist(intake, wrist));

    driverController
        .rightTrigger()
        .onTrue(MainCommands.runIntake(intake))
        .onFalse(MainCommands.stopIntake(intake));
    driverController
        .leftTrigger()
        .onTrue(MainCommands.runOuttake(intake))
        .onFalse(MainCommands.stopIntake(intake));
    driverController
        .rightBumper()
        .onTrue(MainCommands.setClimberDown(climber))
        .onFalse(MainCommands.stopClimber(climber));
    driverController
        .leftBumper()
        .onTrue(MainCommands.setClimberUp(climber))
        .onFalse(MainCommands.stopClimber(climber));

    operatorController
        .povDown()
        .onTrue(
            MainCommands.setElevatorPosition(
                wrist, elevator, ElevatorConstants.L1, WristConstants.SCORING_POSITION_L1));
    operatorController
        .povLeft()
        .onTrue(
            MainCommands.setElevatorPosition(
                wrist, elevator, ElevatorConstants.L2, WristConstants.SCORING_POSITION_L2));
    operatorController
        .povUp()
        .onTrue(
            MainCommands.setElevatorPosition(
                wrist, elevator, ElevatorConstants.L3, WristConstants.SCORING_POSITION_L2));
    operatorController
        .povRight()
        .onTrue(
            MainCommands.setElevatorPosition(
                wrist, elevator, ElevatorConstants.L4, WristConstants.SCORING_POSITION_L4));
    // operatorController
    //     .b()
    //     .onTrue(
    //         MainCommands.setElevatorPosition(
    //             wrist, elevator, ElevatorConstants.NET, WristConstants.SCORING_POSITION_NET));
    operatorController.a().onTrue(MainCommands.setIntakePosition(wrist, elevator));
    operatorController
        .x()
        .onTrue(MainCommands.runOuttakeSlow(intake))
        .onFalse(MainCommands.stopIntake(intake));
    operatorController.y().onTrue(MainCommands.stow(wrist, elevator));
    operatorController
        .rightTrigger()
        .onTrue(MainCommands.runIntake(intake))
        .onFalse(MainCommands.stopIntake(intake));
    operatorController
        .leftTrigger()
        .onTrue(MainCommands.runOuttake(intake))
        .onFalse(MainCommands.stopIntake(intake));
    operatorController.rightBumper().onTrue(MainCommands.changeWristSetpoint(wrist, -.5));
    operatorController.leftBumper().onTrue(MainCommands.changeWristSetpoint(wrist, .5));
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
