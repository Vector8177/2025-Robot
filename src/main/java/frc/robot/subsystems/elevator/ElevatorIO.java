package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    double leftElevatorVelocityRadPerSec = 0d;
    double rightElevatorVelocityRadPerSec = 0d;
    double leftElevatorAppliedVolts = 0d;
    double rightElevatorAppliedVolts = 0d;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  // Sets the power to the elevator motor
  public default void setElevatorVoltage(double voltage) {}

  // Gets the current position of the elevator (in encoder units)
  public default double getPosition() {
    return 0;
  }

  public default void setPosition(double position) {}

  // Resets the encoder position to a specific value
  public default void resetPosition() {}

  public default void stop() {}
}
