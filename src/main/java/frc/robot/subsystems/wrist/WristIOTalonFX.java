package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.WristConstants;

public class WristIOTalonFX implements WristIO {
  private final TalonFX talon;
  private final TalonFXConfiguration configuration;

  public WristIOTalonFX() {
    talon = new TalonFX(WristConstants.wristMotorId); // Change ID to correct one
    configuration = new TalonFXConfiguration(); // Change ID to correct one

    talon.getConfigurator().apply(this.configuration, .05);

    talon.setPosition(0);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {}

  @Override
  public void resetRelativeEncoder() {
    talon.setPosition(0);
  }

  @Override
  public void setVoltage(double voltage) {
    talon.setVoltage(voltage);
  }

  @Override
  public double getPosition() {
    return talon.getPosition().getValueAsDouble();
  }
}
