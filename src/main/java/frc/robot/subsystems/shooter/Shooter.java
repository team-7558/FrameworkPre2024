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

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends StateMachineSubsystemBase {

  private static Shooter instance;

  public static Shooter getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case REAL:
          // Real robot, instantiate hardware IO implementations
          instance = new Shooter(new ShooterIOReal());
          break;

        case SIM:
          // Sim robot, instantiate physics sim IO implementations
          instance = new Shooter(new ShooterIOSim());
          break;

        default:
          // Replayed robot, disable IO implementations
          instance = new Shooter(new ShooterIO() {});
          break;
      }
    }

    return instance;
  }

  public final State DISABLED, IDLE, TARGET_LOW, TARGET_HIGH, SHOOT;

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  public ShooterSetpoints activeSetpoint = ShooterSetpoints.DEFAULT;
  // hoodangle at 1 rad because angle of hood at max height is around 60 degrees, turret is 180
  // degrees turret so 90 seems like it would be ok

  /** Creates a new Flywheel. */
  private Shooter(ShooterIO io) {
    super("Shooter");
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        io.flywheelConfigurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        io.flywheelConfigurePID(0.5, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }

    DISABLED =
        new State("DISABLED") {

          @Override
          public void init() {
            stop();
            // stop hood
            // stop target angle spining
          }

          @Override
          public void periodic() {}

          @Override
          public void exit() {}
        };

    IDLE =
        new State("IDLE") {

          @Override
          public void init() {
            stop();
          }

          @Override
          public void periodic() {}

          @Override
          public void exit() {}
        };

    TARGET_LOW =
        new State("TARGET_LOW") {

          @Override
          public void init() {}

          @Override
          public void periodic() {}

          @Override
          public void exit() {}
        };

    TARGET_HIGH =
        new State("TARGET_HIGH") {

          @Override
          public void init() {}

          @Override
          public void periodic() {
            target(activeSetpoint);
            // hood position
            // target angle
          }

          @Override
          public void exit() {}
        };

    SHOOT =
        new State("SHOOT") {

          @Override
          public void init() {}

          @Override
          public void periodic() {}

          @Override
          public void exit() {}
        };

    setCurrentState(DISABLED);
  }

  @Override
  public void inputPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }

  @Override
  public void outputPeriodic() {

    // Log flywheel speed in RPM
    Logger.recordOutput("FlywheelSpeedRPM", getVelocityRPM());
  }

  /** Run closed loop at the specified velocity. */
  public void runFlywheelAtVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setFlywheelVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("FlywheelSetpointRPM", velocityRPM);
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }

  /** Returns the current velocity in RPM. */
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.flywheelVelocityRadPerSec);
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    io.setFlywheelVoltage(volts);
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return inputs.flywheelVelocityRadPerSec;
  }

  public void target(ShooterSetpoints s) {
    runFlywheelAtVelocity(s.rpm);
    // hood Angle
    // target Angle
  }
}
