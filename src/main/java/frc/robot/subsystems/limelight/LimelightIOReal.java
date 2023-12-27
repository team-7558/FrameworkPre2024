// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightIOReal implements LimelightIO {

  NetworkTable limelight;

  public LimelightIOReal(String name) {
    limelight = NetworkTableInstance.getDefault().getTable(name);
  }

  @Override
  public void updateInputs(LimelightIOInputs inputs) {

    double[] botpose_double = limelight.getEntry("botpose").getDoubleArray(new double[7]);
    inputs.botpose =
        new Pose2d(botpose_double[0], botpose_double[1], Rotation2d.fromDegrees(botpose_double[5]));
    inputs.tl = botpose_double[6];
    inputs.tid = limelight.getEntry("tid").getDouble(-1);
    inputs.tx = limelight.getEntry("tx").getDouble(0);
    inputs.ty = limelight.getEntry("ty").getDouble(0);
    inputs.cl = limelight.getEntry("cl").getDouble(0);
    inputs.tv = limelight.getEntry("tv").getDouble(0);
    inputs.pl = limelight.getEntry("pl").getDouble(0);
    inputs.ta = limelight.getEntry("ta").getDouble(0);
    inputs.cm = limelight.getEntry("cm").getDouble(0);
    inputs.ledMode = limelight.getEntry("ledMode").getDouble(0);
  }

  @Override
  public void setDriverCam(double driverCam) {
    limelight.getEntry("cm").setDouble(driverCam);
  }

  @Override
  public void setLED(boolean isTrue) {
    limelight.getEntry("ledMode").setDouble(isTrue ? 1 : 0);
  }

  @Override
  public void setPipeline(double pipeline) {
    limelight.getEntry("pl").setDouble(pipeline);
  }
}
