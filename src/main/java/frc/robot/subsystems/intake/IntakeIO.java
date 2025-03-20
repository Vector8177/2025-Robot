package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  default void updateInputs(IntakeIOInputs inputs) {}

  default void setIntakeVoltage(double volts) {}

  default void stop() {}

  @AutoLog
  class IntakeIOInputs {
    double intakeVelocityRadPerSec = 0d;
    double intakeAppliedVolts = 0d;
  }
}
