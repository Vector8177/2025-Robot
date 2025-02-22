package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX leadMotor;
  private final TalonFX followMotor;
  private final TalonFXConfiguration config;
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

  // Constructor
  public ElevatorIOTalonFX() {
    leadMotor = new TalonFX(ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID);
    followMotor = new TalonFX(ElevatorConstants.RIGHT_ELEVATOR_MOTOR_ID);
    config = new TalonFXConfiguration();

    leadMotor.getConfigurator().apply(config, 0.05);
    followMotor.getConfigurator().apply(config, 0.05);

    followMotor.setControl(new Follower(ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID, true));

    leadMotor.setPosition(0);
    followMotor.setPosition(0);
  }

  @Override
  public void setElevatorVoltage(double voltage) {
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
    leadMotor.setPosition(0.0);
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
