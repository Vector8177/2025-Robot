package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final PIDController pidController;
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private double targetPosition;
  private ArmFeedforward feedForward;

  public Wrist(WristIO io) {

    this.io = io;
    pidController = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);
    pidController.setTolerance(.2);

    feedForward =
        new ArmFeedforward(
            WristConstants.kS, WristConstants.kG, WristConstants.kV, WristConstants.kA);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
    Logger.recordOutput("Current Position", io.getPosition());
    double pidMotorSpeed =
        pidController.calculate(io.getPosition(), targetPosition)
            + feedForward.calculate(targetPosition, 0);
    ;
    Logger.recordOutput("Wrist Speed", pidMotorSpeed);
    setMotor(
        MathUtil.clamp((pidMotorSpeed), -WristConstants.MAX_VOLTAGE, WristConstants.MAX_VOLTAGE));
  }

  public void setMotor(double voltage) {
    io.setVoltage(voltage);
  }

  public void setPosition(double position) {
    Logger.recordOutput("Wrist Target Position", position);

    targetPosition = position;
  }

  public void setWristSetpoint(double offset) {
    targetPosition = (io.getPosition() + offset);
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }
}
