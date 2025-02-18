package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX leadMotor;
  private final TalonFX followMotor;
  private final TalonFXConfiguration config;

  // Constructor
  public ElevatorIOTalonFX() {
    leadMotor = new TalonFX(31); // CHANGE IDS LATER
    followMotor = new TalonFX(32);
    config = new TalonFXConfiguration();

    leadMotor.getConfigurator().apply(config, 0.05);
    followMotor.getConfigurator().apply(config, 0.05);

    followMotor.setControl(new Follower(31, true));
  }

  @Override
  public void set(double voltage) {
    leadMotor.setVoltage(voltage);
  }

  @Override
  public double getPosition() {
    return leadMotor.getPosition().getValueAsDouble();
  }

  @Override
  public double getVelocity() {
    return leadMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void resetPosition() {
    // leadMotor.setPosition(0.0);
  }

  @Override
  public void setPosition(double position) {
    leadMotor.setPosition(position);
  }

  @Override
  public void stop() {
    leadMotor.stopMotor();
  }
}
