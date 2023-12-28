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
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.Paths;
import frc.robot.OI;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.subsystems.limelight.LimelightIO;
import frc.robot.subsystems.limelight.LimelightIOInputsAutoLogged;
import frc.robot.subsystems.limelight.LimelightIOReal;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.Util;
import java.io.IOException;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Drive extends StateMachineSubsystemBase {

  // STUFF FOR PATHING STATE

  public static transient PathPlannerPath currentPath = Paths.DriveToKey;
  public static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);
  private static final double TRACK_WIDTH_X = Units.inchesToMeters(25.0);
  private static final double TRACK_WIDTH_Y = Units.inchesToMeters(25.0);
  private static final double SKEW_CONSTANT = 0.06;
  private static final double APRILTAG_COEFFICIENT = 0.01;
  private static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

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
                  new ModuleIOSim(0),
                  new ModuleIOSim(1),
                  new ModuleIOSim(2),
                  new ModuleIOSim(3));
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

  public final State DISABLED, SHOOTING, STRAFE_N_TURN, PATHING, STRAFE_AUTOLOCK;

  private final LimelightIO limelight;
  private final LimelightIOInputsAutoLogged limelightInputs = new LimelightIOInputsAutoLogged();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  private AprilTagFieldLayout aprilTagFieldLayout;
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private SwerveDrivePoseEstimator poseEstimator;
  private double autolockSetpoint = 0;
  private Pose2d estimatedPoseNoGyro = new Pose2d();
  private Rotation2d lastGyroRotation = new Rotation2d();
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);

  private Field2d field = new Field2d();

  private Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {

    super("Drive");
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, Module.Mode.SETPOINT);
    modules[1] = new Module(frModuleIO, 1, Module.Mode.SETPOINT);
    modules[2] = new Module(blModuleIO, 2, Module.Mode.SETPOINT);
    modules[3] = new Module(brModuleIO, 3, Module.Mode.SETPOINT);

    limelight = new LimelightIOReal("limelight");
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
              "Drive/Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Drive/Odometry/TrajectorySetpoint", targetPose);
        });

    try {
      aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
    }

    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            getRotation(),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.05),
            VecBuilder.fill(0.5, 0.5, 0.5)); // TODO: TUNE STANDARD DEVIATIONS

    STRAFE_AUTOLOCK =
        new State("STRAFE AUTOLOCK") {
          @Override
          public void periodic() {
            double throttle = 1.0;
            throttle = Util.lerp(1, 0.2, OI.DR.getRightTriggerAxis());
            double err =
                Math.IEEEremainder(
                    getPose().getRotation().getRadians() - Units.degreesToRadians(autolockSetpoint),
                    Math.PI * 2.0);
            Logger.recordOutput("Drive/Autolock Heading Error", err);
            double con = Util.inRange(err, 0.35) ? 2 * err : 0.8 * err;
            Logger.recordOutput("Drive/Autolock Heading Output", con);
            drive(-OI.DR.getLeftY(), -OI.DR.getLeftX(), -con, throttle);
          }
        };

    DISABLED =
        new State("DISABLED") {

          @Override
          public void init() {
            stop();
          }

          @Override
          public void periodic() {}

          @Override
          public void exit() {}
        };

    SHOOTING =
        new State("SHOOTING") {
          @Override
          public void periodic() {
            stopWithX();
          }
        };

    STRAFE_N_TURN =
        new State("STRAFE N TURN") {
          @Override
          public void periodic() {
            double throttle = 1.0;
            throttle = Util.lerp(1, 0.2, OI.DR.getRightTriggerAxis());
            drive(-OI.DR.getLeftY(), -OI.DR.getLeftX(), -OI.DR.getRightX(), throttle);
          }
        };
    PATHING =
        new State("PATHING") {
          private final Timer timer = new Timer();
          private PathPlannerPath targetPath;
          private Pose2d targetPose;
          private GoalEndState goalEndState;
          private PathConstraints constraints;
          private Supplier<Pose2d> poseSupplier;
          private Supplier<ChassisSpeeds> speedsSupplier;
          private Consumer<ChassisSpeeds> output;
          private PathFollowingController ppSwerveController;
          private double rotationDelayDistance;
          private ReplanningConfig replanningConfig;

          private PathPlannerPath currentPath;
          private PathPlannerTrajectory currentTrajectory;
          private Pose2d startingPose;

          private double timeOffset = 0;

          @Override
          public void init() {

            // go to current target path
            // CommandScheduler.getInstance()
            //     .schedule(
            //         AutoBuilder.pathfindThenFollowPath(
            //             currentPath,
            targetPath = Drive.currentPath;
            this.constraints =
                new PathConstraints(3, 3, MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED * 1.5);
            Pathfinding.ensureInitialized();

            Rotation2d targetRotation = new Rotation2d();
            for (PathPoint p : targetPath.getAllPathPoints()) {
              if (p.holonomicRotation != null) {
                targetRotation = p.holonomicRotation;
                break;
              }
            }

            targetPath = Drive.currentPath;
            targetPose = new Pose2d(targetPath.getPoint(0).position, targetRotation);
            goalEndState =
                new GoalEndState(
                    targetPath.getGlobalConstraints().getMaxVelocityMps(), targetRotation);
            this.ppSwerveController =
                new PPHolonomicDriveController(
                    new PIDConstants(0, 0, 0),
                    new PIDConstants(0, 0, 0),
                    MAX_LINEAR_SPEED / 4,
                    Drive.DRIVE_BASE_RADIUS);
            poseSupplier = () -> getPose();
            this.speedsSupplier = () -> getChassisSpeeds();
            this.output = (velocity) -> runVelocity(velocity);
            this.rotationDelayDistance = 0;
            this.replanningConfig = new ReplanningConfig();

            currentTrajectory = null;
            timeOffset = 0;

            Pose2d currentPose = poseSupplier.get();

            ppSwerveController.reset(currentPose, speedsSupplier.get());

            if (targetPath != null) {
              targetPose =
                  new Pose2d(this.targetPath.getPoint(0).position, goalEndState.getRotation());
            }

            if (currentPose.getTranslation().getDistance(targetPose.getTranslation()) < 0.25) {
              Drive.getInstance().setCurrentState(Drive.getInstance().STRAFE_N_TURN);
            } else {
              Pathfinding.setStartPosition(currentPose.getTranslation());
              Pathfinding.setGoalPosition(targetPose.getTranslation());
            }

            startingPose = currentPose;
          }

          @Override
          public void periodic() {
            if (targetPath != null) {
              Pose2d currentPose = poseSupplier.get();
              ChassisSpeeds currentSpeeds = speedsSupplier.get();

              double currentVel =
                  Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
              double stoppingDistance =
                  Math.pow(currentVel, 2) / (2 * constraints.getMaxAccelerationMpsSq());

              if (currentPose.getTranslation().getDistance(targetPath.getPoint(0).position)
                  <= stoppingDistance) {
                Drive.getInstance().setCurrentState(Drive.getInstance().STRAFE_N_TURN);
              }
            }

            if (currentTrajectory != null) {
              if (timer.hasElapsed(currentTrajectory.getTotalTimeSeconds() - timeOffset)) {
                Drive.getInstance().setCurrentState(Drive.getInstance().STRAFE_N_TURN);
              }
            }

            Pose2d currentPose = poseSupplier.get();
            ChassisSpeeds currentSpeeds = speedsSupplier.get();

            PathPlannerLogging.logCurrentPose(currentPose);
            PPLibTelemetry.setCurrentPose(currentPose);

            if (Pathfinding.isNewPathAvailable()) {
              currentPath = Pathfinding.getCurrentPath(constraints, goalEndState);

              if (currentPath != null) {
                ChassisSpeeds fieldRelativeSpeeds =
                    ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, currentPose.getRotation());
                Rotation2d currentHeading =
                    new Rotation2d(
                        fieldRelativeSpeeds.vxMetersPerSecond,
                        fieldRelativeSpeeds.vyMetersPerSecond);
                Rotation2d headingError =
                    currentHeading.minus(currentPath.getStartingDifferentialPose().getRotation());
                boolean onHeading =
                    Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)
                            < 0.5
                        || Math.abs(headingError.getDegrees()) < 30;

                if (!replanningConfig.enableInitialReplanning
                    || (currentPose.getTranslation().getDistance(currentPath.getPoint(0).position)
                            <= 0.25
                        && onHeading)) {
                  currentTrajectory = new PathPlannerTrajectory(currentPath, currentSpeeds);

                  // Find the two closest states in front of and behind robot
                  int closestState1Idx = 0;
                  int closestState2Idx = 1;
                  while (true) {
                    double closest2Dist =
                        currentTrajectory
                            .getState(closestState2Idx)
                            .positionMeters
                            .getDistance(currentPose.getTranslation());
                    double nextDist =
                        currentTrajectory
                            .getState(closestState2Idx + 1)
                            .positionMeters
                            .getDistance(currentPose.getTranslation());
                    if (nextDist < closest2Dist) {
                      closestState1Idx++;
                      closestState2Idx++;
                    } else {
                      break;
                    }
                  }

                  // Use the closest 2 states to interpolate what the time offset should be
                  // This will account for the delay in pathfinding
                  var closestState1 = currentTrajectory.getState(closestState1Idx);
                  var closestState2 = currentTrajectory.getState(closestState2Idx);

                  double d = closestState1.positionMeters.getDistance(closestState2.positionMeters);
                  double t =
                      (currentPose.getTranslation().getDistance(closestState1.positionMeters)) / d;

                  timeOffset =
                      GeometryUtil.doubleLerp(
                          closestState1.timeSeconds, closestState2.timeSeconds, t);

                  PathPlannerLogging.logActivePath(currentPath);
                  PPLibTelemetry.setCurrentPath(currentPath);
                } else {
                  PathPlannerPath replanned = currentPath.replan(currentPose, currentSpeeds);
                  currentTrajectory = new PathPlannerTrajectory(replanned, currentSpeeds);

                  timeOffset = 0;

                  PathPlannerLogging.logActivePath(replanned);
                  PPLibTelemetry.setCurrentPath(replanned);
                }

                timer.reset();
                timer.start();
              }
            }

            if (currentTrajectory != null) {
              PathPlannerTrajectory.State targetState =
                  currentTrajectory.sample(timer.get() + timeOffset);

              if (replanningConfig.enableDynamicReplanning) {
                double previousError = Math.abs(ppSwerveController.getPositionalError());
                double currentError =
                    currentPose.getTranslation().getDistance(targetState.positionMeters);

                if (currentError >= replanningConfig.dynamicReplanningTotalErrorThreshold
                    || currentError - previousError
                        >= replanningConfig.dynamicReplanningErrorSpikeThreshold) {
                  replanPath(currentPose, currentSpeeds);
                  timer.reset();
                  targetState = currentTrajectory.sample(0);
                }
              }

              // Set the target rotation to the starting rotation if we have not yet traveled the
              // rotation
              // delay distance
              if (currentPose.getTranslation().getDistance(startingPose.getTranslation())
                  < rotationDelayDistance) {
                targetState.targetHolonomicRotation = startingPose.getRotation();
              }

              ChassisSpeeds targetSpeeds =
                  ppSwerveController.calculateRobotRelativeSpeeds(currentPose, targetState);

              double currentVel =
                  Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

              PPLibTelemetry.setCurrentPose(currentPose);
              PathPlannerLogging.logCurrentPose(currentPose);

              if (ppSwerveController.isHolonomic()) {
                PPLibTelemetry.setTargetPose(targetState.getTargetHolonomicPose());
                PathPlannerLogging.logTargetPose(targetState.getTargetHolonomicPose());
              } else {
                PPLibTelemetry.setTargetPose(targetState.getDifferentialPose());
                PathPlannerLogging.logTargetPose(targetState.getDifferentialPose());
              }

              PPLibTelemetry.setVelocities(
                  currentVel,
                  targetState.velocityMps,
                  currentSpeeds.omegaRadiansPerSecond,
                  targetSpeeds.omegaRadiansPerSecond);
              PPLibTelemetry.setPathInaccuracy(ppSwerveController.getPositionalError());

              output.accept(targetSpeeds);
            }
          }

          @Override
          public void exit() {
            timer.stop();

            // Only output 0 speeds when ending a path that is supposed to stop, this allows
            // interrupting
            // the command to smoothly transition into some auto-alignment routine
            output.accept(new ChassisSpeeds());
          }

          private void replanPath(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
            PathPlannerPath replanned = currentPath.replan(currentPose, currentSpeeds);
            currentTrajectory = new PathPlannerTrajectory(replanned, currentSpeeds);
            PathPlannerLogging.logActivePath(replanned);
            PPLibTelemetry.setCurrentPath(replanned);
          }
        };

    setCurrentState(DISABLED);
  }

  @Override
  public void inputPeriodic() {
    gyroIO.updateInputs(gyroInputs);
    limelight.updateInputs(limelightInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    Logger.processInputs("Drive/Limelight", limelightInputs);
    for (var module : modules) {
      module.inputPeriodic();
    }
  }

  @Override
  public void outputPeriodic() {
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
    Logger.recordOutput("Drive/SwerveStates/Measured", getModuleStates());

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("Drive/SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
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

    estimatedPoseNoGyro = estimatedPoseNoGyro.exp(twist);
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getRotation(), getModulePositions());

    // TODO: not tested on real bot

    // check to see if the limelight is connected & has a target
    if (limelightInputs.tid != -1) {

      // get predetermined pose of apriltag
      Pose2d tagPose2d = aprilTagFieldLayout.getTagPose((int) limelightInputs.tid).get().toPose2d();

      Pose2d pose = limelightInputs.botpose;

      // TODO: tune vision std devs
      // gyro rotation usually will not be inaccurate enough to have an effect on odometry, so it
      // gets a high number also cause vision rotation is not very accurate
      // x and y will scale based on the distance from the tag * a coefficient that needs to be
      // tuned
      Matrix<N3, N1> stdDevs =
          VecBuilder.fill(
              pose.relativeTo(tagPose2d).getTranslation().getNorm() * APRILTAG_COEFFICIENT,
              pose.relativeTo(tagPose2d).getTranslation().getNorm() * APRILTAG_COEFFICIENT,
              5.0);

      // get latency in seconds
      double latency = limelightInputs.tl;

      // timestamp of limelight data including latency
      double timestamp = Timer.getFPGATimestamp() - latency;

      // adds vision measurement to our odometry only if it's within a meter of the current pose (as
      // reccomended)
      if (pose.relativeTo(getPose()).getTranslation().getNorm() < 1) {
        poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
      }
    }

    Logger.recordOutput("Drive/Odometry/Robot", getPose());
    chassisSpeeds = kinematics.toChassisSpeeds(getModuleStates());
    field.setRobotPose(getPose());
    SmartDashboard.putData(field);
  }

  public void drive(double x, double y, double w, double throttle) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), Constants.driveDeadband);
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
            (linearVelocity.getX() * MAX_LINEAR_SPEED) * throttle,
            (linearVelocity.getY() * MAX_LINEAR_SPEED) * throttle,
            omega * MAX_ANGULAR_SPEED,
            getPose()
                .getRotation()
                .plus(
                    new Rotation2d(
                        getAngularVelocity() * SKEW_CONSTANT)))); // TODO: tune skew constant
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
    Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", optimizedSetpointStates);
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

  /**
   * Returns the angular velocity of the robot
   *
   * @return radians/s
   */
  public double getAngularVelocity() {
    if (gyroInputs.connected) {
      return gyroInputs.yawVelocityRadPerSec;
    } else {
      return chassisSpeeds.omegaRadiansPerSecond;
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
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    if (gyroInputs.connected) {
      return gyroInputs.yawPosition;
    } else {
      return estimatedPoseNoGyro.getRotation();
    }
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(lastGyroRotation, getModulePositions(), pose);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      modules[0].getPosition(),
      modules[1].getPosition(),
      modules[2].getPosition(),
      modules[3].getPosition()
    };
  }

  /** Returns robot relative chassis speeds * */
  public ChassisSpeeds getChassisSpeeds() {
    return chassisSpeeds;
  }

  /** Returns field relative chassis speeds * */
  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond,
        getRotation());
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

  /**
   * Sets the autolock setpoint
   *
   * @param degrees
   */
  public void setAutolockSetpoint(double degrees) {
    this.autolockSetpoint = degrees;
  }

  /**
   * Gets the autolock setpoint
   *
   * @return degrees
   */
  public double getAutolockSetpoint() {
    return autolockSetpoint;
  }

  public void setModuleModes(Module.Mode mode) {
    modules[0].mode = mode;
    modules[1].mode = mode;
    modules[2].mode = mode;
    modules[3].mode = mode;
  }
}
