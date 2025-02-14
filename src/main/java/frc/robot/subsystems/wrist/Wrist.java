package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.WristConstants.PIDFFmode;

public class Wrist extends SubsystemBase {
  private double targetPosition;
  private final PIDController pidController;
  private ArmFeedforward feedForward;

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  public Wrist(WristIO io) {

    this.io = io;
    pidController =
        new PIDController(
            WristConstants.unweightedP, WristConstants.unweightedI, WristConstants.unweightedD);
    pidController.enableContinuousInput(0, Math.PI * 2);
    pidController.setTolerance(.25);

    feedForward =
        new ArmFeedforward(
            WristConstants.unweightedS,
            WristConstants.unweightedG,
            WristConstants.unweightedV,
            WristConstants.unweightedA);

    // setPosition(Position.STANDBY.getWrist());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // Logger.getInstance().processInputs("Wrist", inputs);

    double pidMotorSpeed =
        pidController.calculate(inputs.absoluteEncoderPosition, targetPosition)
            + feedForward.calculate(targetPosition, 0);
    setMotor(
        MathUtil.clamp(
            (pidMotorSpeed), -WristConstants.maxMotorVoltage, WristConstants.maxMotorVoltage));
  }

  public void setPIDFFMode(PIDFFmode mode) {
    pidController.setPID(mode.kP, mode.kI, mode.kD);
    feedForward = new ArmFeedforward(mode.kS, mode.kG, mode.kV, mode.kA);
    // if (mode == PIDFFmode.WEIGHTED) {
    //     Logger.getInstance().recordOutput("WristPIDMode", "Weighted");
    // } else {
    //     Logger.getInstance().recordOutput("WristPIDMode", "Unweighted");
    // }
  }

  public double getEncoderPosition() {
    return inputs.absoluteEncoderPosition;
  }

  public void setMotor(double voltage) {
    io.setVoltage(voltage);
  }

  public Command moveWrist(double position) {
    return run(() -> setPosition(position)).until(() -> atSetpoint());
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
