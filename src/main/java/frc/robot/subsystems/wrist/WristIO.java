package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public double wristVelocityRadPerSec = 0.0;
    public double wristAbsoluteEncoderPosition = 0.0;
    public double wristAppliedVolts = 0.0;
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void setVoltage(double speed) {}

  public default void resetRelativeEncoder() {}

  public default double getPosition() {
    return 0;
  }
}
