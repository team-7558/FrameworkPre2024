// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class ShooterIOReal implements ShooterIO {
  private static final double GEAR_RATIO = 1.5;

  private final TalonFX flywheel = new TalonFX(11);
  private final TalonFX turret = new TalonFX(12);

  private final StatusSignal<Double> flywheelPosition = flywheel.getPosition();
  private final StatusSignal<Double> fywheelVelocity = flywheel.getVelocity();
  private final StatusSignal<Double> flywheelAppliedVolts = flywheel.getMotorVoltage();
  private final StatusSignal<Double> flywheelCurrent = flywheel.getStatorCurrent();
  private final StatusSignal<Double> turretPosition = turret.getPosition();
  private final StatusSignal<Double> turretVelocity = turret.getVelocity();
  private final StatusSignal<Double> turretAppliedVolts = turret.getMotorVoltage();
  private final StatusSignal<Double> turretCurrent = turret.getStatorCurrent();

  public ShooterIOReal() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheel.getConfigurator().apply(config);
    turret.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, flywheelPosition, fywheelVelocity, flywheelAppliedVolts, flywheelCurrent);
    flywheel.optimizeBusUtilization();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, turretPosition, turretVelocity, turretAppliedVolts, turretCurrent);
    turret.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        flywheelPosition, fywheelVelocity, flywheelAppliedVolts, flywheelCurrent);
    inputs.hoodPositionRad =
        Units.rotationsToRadians(flywheelPosition.getValueAsDouble()) / GEAR_RATIO;
    inputs.flywheelVelocityRadPerSec =
        Units.rotationsToRadians(fywheelVelocity.getValueAsDouble()) / GEAR_RATIO;
    inputs.flywheelAppliedVolts = flywheelAppliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {flywheelCurrent.getValueAsDouble()};
  }

  @Override
  public void setFlywheelVoltage(double volts) {
    flywheel.setControl(new VoltageOut(volts));
  }

  @Override
  public void setFlywheelVelocity(double velocityRadPerSec, double ffVolts) {
    flywheel.setControl(
        new VelocityVoltage(
            Units.radiansToRotations(velocityRadPerSec), 0.0, true, ffVolts, 0, false));
  }

  @Override
  public void stop() {
    flywheel.stopMotor();
    turret.stopMotor();
  }

  @Override
  public void flywheelConfigurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    flywheel.getConfigurator().apply(config);
  }

  @Override
  public void turretConfigurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    turret.getConfigurator().apply(config);
  }

  @Override
  public void setTurretAngle() {}
}
