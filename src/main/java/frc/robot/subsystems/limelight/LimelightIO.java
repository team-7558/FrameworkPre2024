// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface LimelightIO {

  @AutoLog
  public static class LimelightIOInputs {
    public double tv = 0;
    public double ta = 0;
    public double tx = 0;
    public double ty = 0;
    public double cm = 0;
    public double pl = 0;
    public double tl = 0;
    public double cl = 0;
    public double ledMode = 0;
    public Pose2d botpose = new Pose2d();
    public double tid = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(LimelightIOInputs inputs) {}

  public default void setLED(boolean isTrue) {}

  public default void setPipeline(double pipeline) {}

  public default void setDriverCam(double driverCam) {}
}
