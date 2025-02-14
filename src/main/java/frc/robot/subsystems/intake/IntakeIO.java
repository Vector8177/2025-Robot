package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  default void updateInputs(IntakeIOInputs inputs) {}

  default void setIntakeVoltage(double volts) {}

  default void runVolts(double volts) {}

  default void stop() {}

  @AutoLog
  class IntakeIOInputs {
    double intakeRadPerSec = 0d;
    double intakeAppliedVolts = 0d;
    double[] intakeCurrentAmps = new double[] {};
  }
}
