package frc.robot.subsystems;

import java.util.logging.Logger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  private final Logger logger;
  private final NetworkTable table;
  private final NetworkTableEntry ty;
  private final NetworkTableEntry tv;

  private double estimatedDistance;

  /** Creates a new Limelight . */
  public Limelight() {
    logger = Logger.getLogger(Limelight.class.getName());
    table = NetworkTableInstance.getDefault().getTable("limelight");
    ty = table.getEntry("ty");
    tv = table.getEntry("tv");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (tv.getBoolean(false)) {
      double angleToGoalDegrees = Constants.Subsystem.Limelight.MOUNT_ANGLE + ty.getDouble(0);
      double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

      // calculate distance
      double distanceFromLimelightToGoalInches = (Constants.Subsystem.Limelight.HIGH_GOAL_HEIGHT
          - Constants.Subsystem.Limelight.MOUNT_HEIGHT) / Math.tan(angleToGoalRadians);

      estimatedDistance = distanceFromLimelightToGoalInches * 0.0254;
    }
  }

  /**
   * Returns the estimated distance to target in meters, or null if there's no target in range.
   */
  public Double distanceToTarget() {
    if (!tv.getBoolean(false)) {
      return null;
    }
    return estimatedDistance;
  }
}
