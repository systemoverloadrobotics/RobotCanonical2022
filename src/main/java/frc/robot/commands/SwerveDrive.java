package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SwerveDrive extends CommandBase {

  private final Swerve swerve;
  private DoubleSupplier xSupplier, ySupplier, rotationSupplier;
  private SlewRateLimiter xLimiter, yLimiter, rotationLimiter;// if the joystick is moved violently

  public SwerveDrive(Swerve swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier) {
    this.swerve = swerve;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotationSupplier = rotationSupplier;
    xLimiter = new SlewRateLimiter(Constants.Subsystem.Swerve.SWERVE_MAX_SPEED);
    yLimiter = new SlewRateLimiter(Constants.Subsystem.Swerve.SWERVE_MAX_SPEED);
    rotationLimiter = new SlewRateLimiter(Constants.Subsystem.Swerve.SWERVE_ROTATION_MAX_SPEED);
    addRequirements(swerve);
  }

  // Called when the command is first scheduled.
  @Override
  public void initialize() {}

  // Called at 50hz while the command is scheduled.
  @Override
  public void execute() {
    // get joystick inputs
    double xSpeed = ySupplier.getAsDouble();
    double ySpeed = -xSupplier.getAsDouble();
    double rotationSpeed = rotationSupplier.getAsDouble();

    // apply deadband
    double deadband = Constants.Subsystem.Swerve.SWERVE_DEADBAND;
    xSpeed = Math.abs(xSpeed) > deadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > deadband ? ySpeed : 0.0;
    rotationSpeed = Math.abs(rotationSpeed) > deadband ? rotationSpeed : 0.0;

    // squared graph
    boolean xBoolean = xSpeed < 0;
    // SmartDashboard.putNumber("x-exponent", Math.pow(xSpeed, 1.7));
    // SmartDashboard.putNumber("y-exponent", Math.pow(ySpeed, 1.7));
    xSpeed = Math.pow(xSpeed, 2);
    if (xBoolean) {
      xSpeed = -xSpeed;
    }
    boolean yBoolean = ySpeed < 0;
    ySpeed = Math.pow(ySpeed, 2);
    if (yBoolean) {
      ySpeed = -ySpeed;
    }
    boolean rotationBoolean = rotationSpeed < 0;
    rotationSpeed = Math.pow(rotationSpeed, 2);

    if (rotationBoolean) {
      rotationSpeed = -rotationSpeed;
    }

    // smooth driving
    xSpeed = xLimiter.calculate(xSpeed) * Constants.Subsystem.Swerve.SWERVE_MAX_SPEED;
    ySpeed = yLimiter.calculate(ySpeed) * Constants.Subsystem.Swerve.SWERVE_MAX_SPEED;
    rotationSpeed =
        rotationLimiter.calculate(rotationSpeed) * Constants.Subsystem.Swerve.SWERVE_ROTATION_MAX_SPEED;

    // construct chassis
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed,
        rotationSpeed, swerve.getRotation2d());
    // convert to states from the chassis
    SwerveModuleState[] moduleState =
        Constants.RobotDimensions.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    swerve.setModuleStates(moduleState);
  }

  // Called once when the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
