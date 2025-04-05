// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.IntakeConstants;

/** Generic roller IO implementation for a roller or series of rollers using a Kraken. */
public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX intakeMotor;
  private final NeutralOut neutralOut = new NeutralOut();

  public IntakeIOTalonFX() {
    intakeMotor = new TalonFX(IntakeConstants.MOTOR_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotionMagic.MotionMagicAcceleration = 100;

    intakeMotor.getConfigurator().apply(config, 0.5);

    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeAppliedVolts =
        intakeMotor.getDutyCycle().getValueAsDouble()
            * intakeMotor.getSupplyVoltage().getValueAsDouble();
    inputs.intakeVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(intakeMotor.getVelocity().getValueAsDouble());
  }

  @Override
  public void setIntakeVoltage(double volts) {
    intakeMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    intakeMotor.setControl(neutralOut);
  }
}
