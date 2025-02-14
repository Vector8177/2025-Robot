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
    // Initialize the CANSparkMax motors for main and follower
    leadMotor = new TalonFX(99); // CHANGE IDS LATER
    followMotor = new TalonFX(98);
    config = new TalonFXConfiguration();

    leadMotor.getConfigurator().apply(config, 0.05);
    followMotor.getConfigurator().apply(config, 0.05);

    followMotor.setControl(new Follower(99, false));
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
    // encoders
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
