package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  class WristIOInputs {
    public double wristVelocityRadPerSec = 0.0;
    public double wristAbsoluteEncoderPosition = 0.0;
    public double wristAppliedVolts = 0.0;
  }

  default void updateInputs(WristIOInputs inputs) {}

  default void setPosition(double position) {}

  default void setVoltage(double speed) {}

  default double getPosition() {
    return 0;
  }
}
