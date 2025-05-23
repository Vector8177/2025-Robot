package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private double targetPosition;
  private final PIDController pidController;
  private ArmFeedforward feedForward;

  public Elevator(ElevatorIO io) {
    this.io = io;
    pidController =
        new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    pidController.setTolerance(1);
    io.resetPosition();
    feedForward =
        new ArmFeedforward(
            ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    Logger.recordOutput("Elevator Position", io.getPosition());

    double pidMotorSpeed =
        pidController.calculate(io.getPosition(), targetPosition)
            + feedForward.calculate(targetPosition, 0);
    Logger.recordOutput("PID Speed", pidMotorSpeed);

    setMotor(
        MathUtil.clamp(
            (pidMotorSpeed), -ElevatorConstants.MAX_VOLTAGE, ElevatorConstants.MAX_VOLTAGE));
  }

  public void setElevatorSetpoint(double offset) {
    targetPosition = (io.getPosition() + offset);
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  public void setPosition(double position) {
    Logger.recordOutput("ElevatorTargetPosition", position);
    targetPosition = position;
  }

  public double getPosition() {
    return io.getPosition();
  }

  public void setMotor(double voltage) {
    io.setElevatorVoltage(voltage);
  }

  public void resetPosition() {
    io.resetPosition();
  }
}
