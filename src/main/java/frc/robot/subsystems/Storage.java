// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.manager.StorageManager;

public class Storage extends SubsystemBase {
  private final Logger logger;

  private final StorageManager manager;

  /** Creates a new Storage. */
  public Storage(StorageManager manager) {
    logger = Logger.getLogger(ExampleSubsystem.class.getName());
    
    this.manager = manager;
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
