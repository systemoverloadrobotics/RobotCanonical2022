package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.modules.SwerveModule;


public class Swerve extends SubsystemBase {
  //Swerve Modules
  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  //Gyro
  private AHRS gyro = new AHRS(SerialPort.Port.kUSB);
  //Odometer
  private SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(Constants.RobotDimensions.SWERVE_DRIVE_KINEMATICS, new Rotation2d(0));

  private final Timer resetTimer = new Timer();
  private boolean resetRun;

  public Swerve() {
    // Create four modules with correct controllers, add to modules
    frontLeft = new SwerveModule("FrontLeft", Constants.Motor.SWERVE_FRONT_LEFT_POWER,
        Constants.Motor.SWERVE_FRONT_LEFT_STEER, 2389 - 3);
    frontRight = new SwerveModule("FrontRight", Constants.Motor.SWERVE_FRONT_RIGHT_POWER,
        Constants.Motor.SWERVE_FRONT_RIGHT_STEER, 805 - 2);
    backLeft = new SwerveModule("BackLeft", Constants.Motor.SWERVE_BACK_LEFT_POWER,
        Constants.Motor.SWERVE_BACK_LEFT_STEER, 478 + 5);
    backRight = new SwerveModule("BackRight", Constants.Motor.SWERVE_BACK_RIGHT_POWER,
        Constants.Motor.SWERVE_BACK_RIGHT_STEER, 1421 + 30);

    resetTimer.start();
  }

  public void stopModules(){
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(gyro.getYaw());
  }

  public SwerveModuleState[] getModuleStates(){
    return new SwerveModuleState[] {
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()
    };
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    frontLeft.setState(desiredStates[0]);
    frontRight.setState(desiredStates[1]);
    backLeft.setState(desiredStates[2]);
    backRight.setState(desiredStates[3]);
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("Gyro", gyro);

    // Run reset once after a second.
    if (!resetRun && resetTimer.get() > 1) {
      gyro.reset();
      resetRun = true;
    }
    
    odometry.update(getRotation2d(), getModuleStates());
  }
}
