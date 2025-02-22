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
    return runOnce(
        () -> {
          intake.setSpeed(0);
        },
        intake);
  }

  public static Command runIntake(Intake intake) {
    return runOnce(
        () -> {
          intake.setSpeed(1);
        },
        intake);
  }

  public static Command runOutake(Intake intake) {
    return runOnce(
        () -> {
          intake.setSpeed(-1);
        },
        intake);
  }

  public static Command setIntakePosition(Wrist wrist, Elevator elevator) {
    return sequence(
        runOnce(
            () -> {
              elevator.setPosition(ElevatorConstants.ELEVATOR_INTAKE);
            },
            elevator),
        waitSeconds(0.5),
        runOnce(
            () -> {
              wrist.setPosition(WristConstants.WIRST_INTAKE_POSITION);
            },
            wrist));
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

  public static Command setElevatorPosition(
      Wrist wrist, Elevator elevator, double elevatorPosition, double wristPosition) {
    return sequence(
        runOnce(
          () -> {
            wrist.setPosition(WristConstants.WRIST_PERPENDICULAR_POSITION);
          }, 
          wrist),
        waitSeconds(0.5),
        runOnce(
            () -> {
              elevator.setPosition(elevatorPosition);
            },
            elevator),
        waitSeconds(0.5),
        runOnce(
            () -> {
              wrist.setPosition(wristPosition);
            },
            wrist));
  }

  // public static Command setElevatorPositionTest(Elevator elevator, double elevatorPosition) {
  //   return runOnce(
  //       () -> {
  //         elevator.setPosition(elevatorPosition);
  //       },
  //       elevator);
  // }

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
