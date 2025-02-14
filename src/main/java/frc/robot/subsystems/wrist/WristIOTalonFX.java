package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class WristIOTalonFX implements WristIO {
  private final TalonFX talon;
  private final TalonFXConfiguration configuration;

  public WristIOTalonFX() {
    this.talon = new TalonFX(69); // Change ID to correct one
    this.configuration = new TalonFXConfiguration(); // Change ID to correct one

    this.talon.getConfigurator().apply(this.configuration, .05);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {}

  @Override
  public void resetRelativeEncoder() {
    this.talon.getConfigurator().apply(new TalonFXConfiguration());
  }

  @Override
  public void setVoltage(double voltage) {
    this.talon.setVoltage(voltage);
  }
}
