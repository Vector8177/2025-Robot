package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;

public class MainCommands {

  private MainCommands() {}

  public static Command stopIntake(Intake intake) {
    return runOnce(() -> intake.setSpeed(0), intake);
  }

  public static Command runIntake(Intake intake) {
    return runOnce(
        () -> intake.setSpeed(-.3),
        intake);
  }

  public static Command runOuttake(Intake intake) {
    return runOnce(() -> intake.setSpeed(.5), intake);
  }

  public static Command runOuttakeSlow(Intake intake) {
    return runOnce(() -> intake.setSpeed(.15), intake);
  }

  public static Command flickWrist(Intake intake, Wrist wrist) {
    return sequence(
        runOnce(() -> intake.setSpeed(.25), intake),
        waitSeconds(.25),
        runOnce(() -> wrist.setPosition(WristConstants.FLICK_WRIST_POSITION), wrist),
        runOnce(() -> intake.setSpeed(0), intake));
  }

  public static Command setIntakePosition(Wrist wrist, Elevator elevator) {
    return sequence(
        runOnce(() -> wrist.setPosition(WristConstants.PERPENDICULAR_POSITION), wrist),
        waitSeconds(0.25),
        runOnce(() -> elevator.setPosition(ElevatorConstants.INTAKE), elevator),
        waitSeconds(0.25),
        runOnce(() -> wrist.setPosition(WristConstants.INTAKE_POSITION), wrist));
  }

  // Sets wrist and elevator to 0
  public static Command stow(Wrist wrist, Elevator elevator) {
    return sequence(
        runOnce(() -> wrist.setPosition(WristConstants.PERPENDICULAR_POSITION), wrist),
        waitSeconds(.25),
        runOnce(() -> elevator.setPosition(0.1), elevator),
        waitSeconds(.25),
        runOnce(() -> wrist.setPosition(1), wrist));
  }

  public static Command setClimberDown(Climber climber) {
    return runOnce(() -> climber.setSpeed(1), climber);
  }

  public static Command setClimberUp(Climber climber) {
    return runOnce(() -> climber.setSpeed(-1), climber);
  }

  public static Command stopClimber(Climber climber) {
    return runOnce(() -> climber.setSpeed(0), climber);
  }

  // public static Command setElevatorPosition(
  //     Wrist wrist, Elevator elevator, double elevatorPosition, double wristPosition) {
  //   return sequence(
  //       run(() -> wrist.setPosition(WristConstants.PERPENDICULAR_POSITION), wrist)
  //           .until(() -> wrist.atSetpoint()),
  //       // waitSeconds(.20),
  //       run(() -> elevator.setPosition(elevatorPosition), elevator)
  //           .until(() -> elevator.atSetpoint()),
  //       // waitSeconds(.20),
  //       run(() -> wrist.setPosition(wristPosition), wrist).until(() -> wrist.atSetpoint()));
  // }

  public static Command setElevatorPosition(
      Wrist wrist, Elevator elevator, double elevatorPosition, double wristPosition) {
    return sequence(
        runOnce(() -> wrist.setPosition(WristConstants.PERPENDICULAR_POSITION), wrist),
        waitSeconds(.25),
        runOnce(() -> elevator.setPosition(elevatorPosition), elevator),
        waitSeconds(.25),
        runOnce(() -> wrist.setPosition(wristPosition), wrist));
  }

  // manually move the elevator or wrist
  public static Command changeElevatorSetpoint(Elevator elevator, double offset) {
    return runOnce(() -> elevator.setElevatorSetpoint(offset), elevator);
  }

  public static Command changeWristSetpoint(Wrist wrist, double offset) {
    return runOnce(() -> wrist.setWristSetpoint(offset), wrist);
  }
}
