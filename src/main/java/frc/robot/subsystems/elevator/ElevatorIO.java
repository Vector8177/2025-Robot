package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    double leftElevatorVelocityRadPerSec = 0d;
    double rightElevatorVelocityRadPerSec = 0d;
    double leftElevatorAppliedVolts = 0d;
    double rightElevatorAppliedVolts = 0d;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  // Sets the power to the elevator motor
  default void setElevatorVoltage(double voltage) {}

  // Gets the current position of the elevator (in encoder units)
  default double getPosition() {
    return 0;
  }

  default void setPosition(double position) {}

  // Resets the encoder position to a specific value
  default void resetPosition() {}

  default void stop() {}
}
