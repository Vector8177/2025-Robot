package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX leadMotor;
  private final TalonFX followMotor;

  public ElevatorIOTalonFX() {
    leadMotor = new TalonFX(ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID);
    followMotor = new TalonFX(ElevatorConstants.RIGHT_ELEVATOR_MOTOR_ID);
    TalonFXConfiguration config = new TalonFXConfiguration();

    leadMotor.getConfigurator().apply(config, 0.05);
    followMotor.getConfigurator().apply(config, 0.05);

    followMotor.setControl(new Follower(ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID, true));

    leadMotor.setPosition(0);
    followMotor.setPosition(0);

    leadMotor.setNeutralMode(NeutralModeValue.Brake);
    followMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftElevatorAppliedVolts =
        leadMotor.getDutyCycle().getValueAsDouble()
            * leadMotor.getSupplyVoltage().getValueAsDouble();
    inputs.leftElevatorVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(leadMotor.getVelocity().getValueAsDouble());

    inputs.rightElevatorAppliedVolts =
        followMotor.getDutyCycle().getValueAsDouble()
            * followMotor.getSupplyVoltage().getValueAsDouble();
    inputs.rightElevatorVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(followMotor.getVelocity().getValueAsDouble());
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
  public void resetPosition() {
    leadMotor.setPosition(0.0);
    followMotor.setPosition(0.0);
  }

  @Override
  public void setPosition(double position) {
    leadMotor.setPosition(position);
  }
}
