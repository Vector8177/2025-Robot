package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX leftClimberTalonFX;
  private final TalonFX rightClimberTalonFX;
  private final TalonFXConfiguration configuration;

  public ClimberIOTalonFX() {
    leftClimberTalonFX = new TalonFX(Constants.ClimberConstants.LEFT_MOTOR_ID); // Change later
    rightClimberTalonFX = new TalonFX(Constants.ClimberConstants.RIGHT_MOTOR_ID);

    configuration = new TalonFXConfiguration();
    leftClimberTalonFX.getConfigurator().apply(configuration, .05);
    rightClimberTalonFX.getConfigurator().apply(configuration, .05);

    rightClimberTalonFX.setControl(new Follower(Constants.ClimberConstants.LEFT_MOTOR_ID, true));

    leftClimberTalonFX.setNeutralMode(NeutralModeValue.Brake);
    rightClimberTalonFX.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.leftClimberAppliedVolts =
        leftClimberTalonFX.getDutyCycle().getValueAsDouble()
            * leftClimberTalonFX.getSupplyVoltage().getValueAsDouble();
    inputs.leftClimberVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            leftClimberTalonFX.getVelocity().getValueAsDouble());
    // inputs.leftClimberCurrentAmps = new double[] {leftClimberTalonFX.getOutputCurrent()};
    inputs.leftClimberEncoderPosition = leftClimberTalonFX.getPosition().getValueAsDouble();

    inputs.rightClimberAppliedVolts =
        rightClimberTalonFX.getDutyCycle().getValueAsDouble()
            * rightClimberTalonFX.getSupplyVoltage().getValueAsDouble();
    inputs.rightClimberVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            rightClimberTalonFX.getVelocity().getValueAsDouble());
    // inputs.rightClimberCurrentAmps = new double[] {rightClimberTalonFX.getOutputCurrent()};
    inputs.rightClimberEncoderPosition = rightClimberTalonFX.getPosition().getValueAsDouble();
  }

  @Override
  public void setClimberVoltage(double volts) {
    leftClimberTalonFX.setVoltage(volts);
    rightClimberTalonFX.setVoltage(volts);
  }

  @Override
  public void stop() {
    leftClimberTalonFX.setVoltage(0);
    rightClimberTalonFX.setVoltage(0);
  }
}
