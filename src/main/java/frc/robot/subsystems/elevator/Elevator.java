package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double targetSpeed = 0;

  private double targetPosition;
  private final PIDController pidController;
  private ArmFeedforward feedForward;

  // Constructor
  public Elevator(ElevatorIO io) {
    this.io = io;
    pidController =
        new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    pidController.enableContinuousInput(0, Math.PI * 2);
    pidController.setTolerance(.25);

    feedForward =
        new ArmFeedforward(
            ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);
  }

  // Periodic method called in every cycle (e.g., 20ms)
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.recordOutput("Elevator Position", io.getPosition());
    setSpeedRaw(targetSpeed);

    double pidMotorSpeed =
        pidController.calculate(io.getPosition(), targetPosition)
            + feedForward.calculate(targetPosition, 0);
    setMotor(
        MathUtil.clamp(
            (pidMotorSpeed), -WristConstants.MAX_WRIST_VOLTAGE, WristConstants.MAX_WRIST_VOLTAGE));
  }

  private void setSpeedRaw(double speed) {
    speed = MathUtil.clamp(speed, -1, 1);
    io.setElevatorVoltage(speed * Constants.IntakeConstants.MAX_INTAKE_VOLTAGE);
  }

  public void setSpeed(double speed) {
    targetSpeed = speed;
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  public Command moveElevator(double position) {
    return run(() -> setPosition(position)).until(() -> atSetpoint());
  }

  public void setPosition(double position) {
    // Logger.getInstance().recordOutput("WristTargetPosition", position);
    targetPosition = position;
  }

  public void setMotor(double voltage) {
    io.setElevatorVoltage(voltage);
  }

  public void resetPosition() {
    io.resetPosition();
  }
}
