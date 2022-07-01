// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import java.util.logging.Logger;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Intake extends CommandBase {
  private final Logger logger;
  private final frc.robot.subsystems.Intake intakeSubsystem;
  private final Storage storage;
  private final Shooter shooter;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Intake(Shooter shooter, Storage storage, frc.robot.subsystems.Intake intake) {
    logger = Logger.getLogger(ExampleCommand.class.getName());

    this.shooter = shooter;
    this.storage = storage;
    this.intakeSubsystem = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, storage, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
