// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.logging.Logger;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ManageStorage;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Logger logger;

  // The robot's subsystems and commands are defined here...
  private final Shooter shooter = new Shooter();
  private final Swerve swerve = new Swerve();
  private final Intake intake = new Intake();
  private final Storage storage = new Storage();

  private final Limelight limelight = new Limelight();

  // Simple commands
  private final Command shootCommand = new RunCommand(() -> {
    Double distance = limelight.distanceToTarget();
    if (distance == null) {
      shooter.spoolDefault();
    } else {
      shooter.shoot(distance);
    }
  }, shooter);

  private final Command spoolCommand = new RunCommand(() -> {
    Double distance = limelight.distanceToTarget();
    if (distance == null) {
      shooter.spoolDefault();
    } else {
      shooter.spool(distance);
    }
  }, shooter);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    logger = Logger.getLogger(RobotContainer.class.getName());
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
    swerve.setDefaultCommand(new SwerveDrive(swerve, Constants.Input.SWERVE_X.get(),
        Constants.Input.SWERVE_Y.get(), Constants.Input.SWERVE_ROTATION.get()));
    storage.setDefaultCommand(new ManageStorage(storage, intake, shooter));

    Constants.Input.SHOOT.get().whenHeld(shootCommand);
    Constants.Input.SPOOL.get().whenHeld(spoolCommand);
    Constants.Input.INTAKE.get().whenHeld(new RunCommand(intake::intake, intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
