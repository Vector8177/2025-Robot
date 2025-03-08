package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.WristConstants;

public class WristIOTalonFX implements WristIO {
  private final TalonFX wristMotor;
  private final TalonFXConfiguration configuration;
  private final DigitalInput input;
  private final DutyCycleEncoder encoder;

  public WristIOTalonFX() {
    wristMotor = new TalonFX(WristConstants.WRIST_MOTOR_ID);
    configuration = new TalonFXConfiguration();

    wristMotor.getConfigurator().apply(configuration, .05);
    wristMotor.setPosition(0);
    wristMotor.setNeutralMode(NeutralModeValue.Brake);

    input = new DigitalInput(0);
    encoder = new DutyCycleEncoder(input);
    encoder.setDutyCycleRange(0, .5); // change this later
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.wristAppliedVolts =
        wristMotor.getDutyCycle().getValueAsDouble()
            * wristMotor.getSupplyVoltage().getValueAsDouble();
    inputs.wristVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(wristMotor.getVelocity().getValueAsDouble());
  }

  @Override
  public void resetRelativeEncoder() {
    wristMotor.setPosition(0);
  }

  @Override
  public void setVoltage(double voltage) {
    wristMotor.setVoltage(voltage);
  }

  @Override
  public double getPosition() {
    // return wristMotor.getPosition().getValueAsDouble();
    return encoder.get();
  }

  @Override
  public void setPosition(double position) {
    wristMotor.setPosition(position);
  }
}
