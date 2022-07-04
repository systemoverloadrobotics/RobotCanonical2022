package frc.robot.subsystems;

import java.util.logging.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private final Logger logger;

  /** Creates a new Limelight . */
  public Limelight() {
    logger = Logger.getLogger(ExampleSubsystem.class.getName());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Returns the estimated distance to target, or null if there's no target in range.
   * @return
   */
  public Double distanceToTarget() {
    return 0.0; 
  }
}
