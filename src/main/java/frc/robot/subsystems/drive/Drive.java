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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.Logger;

public class Drive extends StateMachineSubsystemBase {
  private static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);
  private static final double TRACK_WIDTH_X = Units.inchesToMeters(25.0);
  private static final double TRACK_WIDTH_Y = Units.inchesToMeters(25.0);
  private static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  private static Drive instance;

  public static Drive getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case REAL:
          // Real robot, instantiate hardware IO implementations
          instance =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(0),
                  new ModuleIOTalonFX(1),
                  new ModuleIOTalonFX(2),
                  new ModuleIOTalonFX(3));
          break;

        case SIM:
          // Sim robot, instantiate physics sim IO implementations
          instance =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
          break;

        default:
          // Replayed robot, disable IO implementations
          instance =
              new Drive(
                  new GyroIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {});
          break;
      }
    }

    return instance;
  }

  public final State DISABLED, X, DRIVING;

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Pose2d pose = new Pose2d();
  private Rotation2d lastGyroRotation = new Rotation2d();

  private Field2d field = new Field2d();

  private Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {

    super("Drive");
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            MAX_LINEAR_SPEED, DRIVE_BASE_RADIUS, new ReplanningConfig()),
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    DISABLED = new State("DISABLED") {

          @Override
          public void init() {
            stop();
          }

          @Override
          public void periodic() {}

          @Override
          public void exit() {}
        };

    X = new State("X") {
          @Override
          public void periodic() {
            stopWithX();
          }
        };

    DRIVING = new State("DRIVING") {
          @Override
          public void periodic() {
            drive(-OI.DR.getLeftY(), -OI.DR.getLeftX(), -OI.DR.getRightX(), 1.0);
          }
        };

    setCurrentState(DISABLED);
  }

  @Override
  public void inputPeriodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.inputPeriodic();
    }
  }

  @Override
  public void outputPeriodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.outputPeriodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log measured states
    Logger.recordOutput("SwerveStates/Measured", getModuleStates());

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      wheelDeltas[i] = modules[i].getPositionDelta();
    }
    // The twist represents the motion of the robot since the last
    // loop cycle in x, y, and theta based only on the modules,
    // without the gyro. The gyro is always disconnected in simulation.
    var twist = kinematics.toTwist2d(wheelDeltas);
    if (gyroInputs.connected) {
      // If the gyro is connected, replace the theta component of the twist
      // with the change in angle since the last loop cycle.
      twist =
          new Twist2d(
              twist.dx, twist.dy, gyroInputs.yawPosition.minus(lastGyroRotation).getRadians());
      lastGyroRotation = gyroInputs.yawPosition;
    }
    // Apply the twist (change since last loop cycle) to the current pose
    pose = pose.exp(twist);
    Logger.recordOutput("Odometry/Robot", getPose());
    field.setRobotPose(pose);
    SmartDashboard.putData(field);
  }

  public void drive(double x, double y, double w, double throttle) {
    // Apply deadband
    double linearMagnitude =
        throttle * MathUtil.applyDeadband(Math.hypot(x, y), Constants.driveDeadband);
    Rotation2d linearDirection = new Rotation2d(x, y);
    double omega = MathUtil.applyDeadband(w, Constants.driveDeadband);

    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;
    omega = Math.copySign(omega * omega, omega);

    // Calcaulate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

    // Convert to field relative speeds & send command
    runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * MAX_LINEAR_SPEED,
            linearVelocity.getY() * MAX_LINEAR_SPEED,
            omega * MAX_ANGULAR_SPEED,
            pose.getRotation()));
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.globalDelta_sec);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(volts);
    }
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  /** Returns the module states (turn angles and drive velocitoes) for all of the modules. */
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  public Pose2d getPose() {
    return pose;
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return pose.getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    this.pose = pose;
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };
  }
}
