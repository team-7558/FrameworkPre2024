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

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double flywheelVelocityRadPerSec = 0.0;
    public double flywheelAppliedVolts = 0.0;
    public double hoodPositionRad = 0.0;
    public double hoodAppliedVolts = 0.0;
    public double hoodVelocityRadPerSec = 0.0;
    public double turretVelocityRadPerSec = 0.0;
    public double turretAppliedVolts = 0.0;
    public double turretPositionRad = 0.0;
    public double[] currentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setFlywheelVoltage(double volts) {}

  /** Run closed loop at the specified velocity. */
  public default void setFlywheelVelocity(double velocityRadPerSec, double ffVolts) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Set velocity PID constants. */
  public default void flywheelConfigurePID(double kP, double kI, double kD) {}

  public default void turretConfigurePID(double kP, double kI, double kD) {}

  public default void hoodConfigurePID(double kP, double kI, double kD) {}

  public default void setTurretAngle() {}
}
