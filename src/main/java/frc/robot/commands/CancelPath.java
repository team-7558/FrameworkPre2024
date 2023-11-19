// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class CancelPath extends Command {

  Drive drive;

  /**
   * only thing this class does is take the drive requirement which will shut down the pathplanner
   * path run
   */
  public CancelPath(Drive drive) {
    addRequirements(drive);
    this.drive = drive;
  }
}
