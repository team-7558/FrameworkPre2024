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

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.RobotTeleop;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Flywheel flywheel;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        flywheel = new Flywheel(new FlywheelIOSparkMax());
        // drive = new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFX(0),
        // new ModuleIOTalonFX(1),
        // new ModuleIOTalonFX(2),
        // new ModuleIOTalonFX(3));
        // flywheel = new Flywheel(new FlywheelIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        flywheel = new Flywheel(new FlywheelIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        flywheel = new Flywheel(new FlywheelIO() {});
        break;
    }

    drive = Drive.getInstance();

    // Set up named commands for PathPlanner
    NamedCommands.registerCommand(
        "Run Flywheel",
        Commands.startEnd(
            () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up FF characterization routines
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));
    autoChooser.addOption(
        "Flywheel FF Characterization",
        new FeedForwardCharacterization(
            flywheel, flywheel::runCharacterizationVolts, flywheel::getCharacterizationVelocity));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(new RobotTeleop());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
