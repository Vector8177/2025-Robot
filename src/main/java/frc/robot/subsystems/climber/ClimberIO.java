package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    double leftClimberVelocityRadPerSec = 0d;
    double rightClimberVelocityRadPerSec = 0d;
    double leftClimberAppliedVolts = 0d;
    double rightClimberAppliedVolts = 0d;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void setClimberVoltage(double volts) {}

  default void stop() {}
}
