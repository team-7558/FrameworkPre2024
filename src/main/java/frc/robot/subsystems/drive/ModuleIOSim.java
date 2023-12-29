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

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
  private static final double LOOP_PERIOD_SECS = Constants.globalDelta_sec;

  private DCMotorSim driveSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
  private DCMotorSim turnSim = new DCMotorSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004);

  private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);

  private final PIDController driveVelocityPID = new PIDController(2.4, 0, 0, LOOP_PERIOD_SECS);
  private final SimpleMotorFeedforward driveVelocityFF = new SimpleMotorFeedforward(0.0, 2.62);
  private final PIDController turnAnglePID = new PIDController(10.0, 0, 0, LOOP_PERIOD_SECS);

  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;
  private double driveVelocity = 0.0;
  private double turnAngle = 0.0;

  private int index;

  public ModuleIOSim(int index) {
    turnAnglePID.enableContinuousInput(-Math.PI, Math.PI);
    this.index = index;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveSim.update(LOOP_PERIOD_SECS);
    turnSim.update(LOOP_PERIOD_SECS);

    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] {Math.abs(driveSim.getCurrentDrawAmps())};

    inputs.turnAbsolutePosition =
        new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = new double[] {Math.abs(turnSim.getCurrentDrawAmps())};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);
  }

  @Override
  public void setDriveVelocity(double velocity) {
    driveVelocity = MathUtil.clamp(velocity, -Drive.MAX_LINEAR_SPEED, Drive.MAX_LINEAR_SPEED);
    double measuredDriveVelocity = driveSim.getAngularVelocityRadPerSec() * Module.WHEEL_RADIUS;
    double ff = driveVelocityFF.calculate(driveVelocity);
    driveAppliedVolts =
        MathUtil.clamp(
            ff + driveVelocityPID.calculate(measuredDriveVelocity, driveVelocity), -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
    Logger.recordOutput(
        "Drive/Modules/" + Integer.toString(index) + "/MeasuredDriveVel", measuredDriveVelocity);
    Logger.recordOutput(
        "Drive/Modules/" + Integer.toString(index) + "/TargetDriveVel", driveVelocity);
  }

  @Override
  public void setTurnAngle(double rad) {
    turnAngle = rad;
    double measuredTurnAngle =
        turnSim.getAngularPositionRad() + turnAbsoluteInitPosition.getRadians();
    turnAppliedVolts =
        MathUtil.clamp(turnAnglePID.calculate(measuredTurnAngle, turnAngle), -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);
    Logger.recordOutput(
        "Drive/Modules/" + Integer.toString(index) + "/MeasuredTurnAngle", measuredTurnAngle);
    Logger.recordOutput("Drive/Modules/" + Integer.toString(index) + "/TargetTurnAngle", turnAngle);
  }
}
