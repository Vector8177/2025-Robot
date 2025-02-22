package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {
  private double targetPosition;
  private final PIDController pidController;
  private ArmFeedforward feedForward;

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  public Wrist(WristIO io) {

    this.io = io;
    pidController = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);
    pidController.enableContinuousInput(0, Math.PI * 2);
    pidController.setTolerance(.25);

    feedForward =
        new ArmFeedforward(
            WristConstants.kS, WristConstants.kG, WristConstants.kV, WristConstants.kA);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // Logger.getInstance().processInputs("Wrist", inputs);

    double pidMotorSpeed =
        pidController.calculate(io.getPosition(), targetPosition)
            + feedForward.calculate(targetPosition, 0);
    setMotor(
        MathUtil.clamp(
            (pidMotorSpeed), -WristConstants.MAX_WRIST_VOLTAGE, WristConstants.MAX_WRIST_VOLTAGE));
  }

  public double getEncoderPosition() {
    return inputs.absoluteEncoderPosition;
  }

  public void setMotor(double voltage) {
    io.setVoltage(voltage);
  }

  public Command moveWrist(double position) {
    return runOnce(() -> setPosition(position)).until(() -> atSetpoint());
  }

  public void setPosition(double position) {
    // Logger.getInstance().recordOutput("WristTargetPosition", position);
    targetPosition = position;
  }

  public double getPosition() {
    return targetPosition;
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  public void resetRelativeEncoder() {
    io.resetRelativeEncoder();
  }
}
