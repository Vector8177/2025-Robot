package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;

public class MainCommands {

  private MainCommands() {}

  public static Command stopIntake(Intake intake) {
    return runOnce(
        () -> {
          intake.setSpeed(0);
        },
        intake);
  }

  public static Command runIntake(Intake intake) {
    return runOnce(
        () -> {
          intake.setSpeed(.75); // SPEED IS NOT CORRECT, CHECK AGAIN - .5
        },
        intake);
  }

  public static Command runOuttake(Intake intake) {
    return runOnce(
        () -> {
          intake.setSpeed(-.75); // SPEED IS NOT CORRECT, CHECK AGAIN
        },
        intake);
  }

  public static Command setWristIntakePosition(Wrist wrist) {
    return runOnce(
        () -> {
          wrist.setPosition(WristConstants.WIRST_INTAKE_POSITION);
        },
        wrist);
  }

  public static Command setWristScoringPosition(Wrist wrist) {
    return runOnce(
        () -> {
          wrist.setPosition(WristConstants.WRIST_SCORING_POSITION);
        },
        wrist);
  }

  public static Command setClimberUp(Climber climber) {
    return runOnce(
        () -> {
          climber.setSpeed(1);
        },
        climber);
  }

  public static Command setClimberDown(Climber climber) {
    return runOnce(
        () -> {
          climber.setSpeed(-1);
        },
        climber);
  }

  public static Command stopClimber(Climber climber) {
    return runOnce(
        () -> {
          climber.setSpeed(0);
        },
        climber);
  }


  public static Command setElevatorPosition(Wrist wrist, Elevator elevator, double position) {
    return sequence(
      runOnce(
        () -> {
          elevator.setPosition(position);
        },
        elevator),
      waitSeconds(1.0),
      runOnce(
        () -> {
          wrist.setPosition(WristConstants.WRIST_SCORING_POSITION);
        },
        wrist));
  }

  public static Command setElevatorVoltage(Elevator elevator, double speed) {
    return runOnce(
        () -> {
          elevator.setSpeed(speed);
        },
        elevator);
  }

  public static Command stopElevator(Elevator elevator) {
    return runOnce(
        () -> {
          elevator.setSpeed(0);
        },
        elevator);
  }
}
