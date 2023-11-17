// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;

public class RobotTeleop extends Command {

  private final Drive drive;
  private final Shooter shooter;

  /** Creates a new DriveTeleop. */
  public RobotTeleop() {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = Drive.getInstance();
    shooter = Shooter.getInstance();

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setCurrentState(drive.STRAFE_N_TURN);
    shooter.setCurrentState(shooter.IDLE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!drive.isState(drive.DISABLED)) {
      // Drive state logic here
      if (OI.DR.getBButton()) drive.setPose(new Pose2d());

      if (OI.DR.getXButton()) drive.setCurrentState(drive.X);
      else if (OI.DR.getAButton()) {
        drive.setAutolockHeading(0.5 * Math.PI);
        drive.setCurrentState(drive.STRAFE_AUTOLOCK);

      } else if (OI.DR.getYButton()) {
        drive.setAutolockHeading(Math.PI);
        drive.setCurrentState(drive.STRAFE_AUTOLOCK);
      } else drive.setCurrentState(drive.STRAFE_N_TURN);
    }

    if (!shooter.isState(shooter.DISABLED)) {
      // Shooter state logic here
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setCurrentState(drive.DISABLED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
