package frc.robot.subsystems;

import java.util.logging.Logger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.sorutil.interpolate.Interpolator;
import frc.sorutil.motor.MotorConfiguration;
import frc.sorutil.motor.SensorConfiguration;
import frc.sorutil.motor.SuMotor;
import frc.sorutil.motor.SuSparkMax;
import frc.sorutil.motor.SuMotor.ControlMode;
import frc.sorutil.motor.SuMotor.IdleMode;

public class Shooter extends SubsystemBase {
  private final SuMotor<SuSparkMax> shooterMotor;

  private final Logger logger;

  private final Interpolator shotSpeed;

  public Shooter() {
    logger = Logger.getLogger(ExampleSubsystem.class.getName());

    // Shooter uses integrated sensor on the SparkMax with no gear reduction
    SensorConfiguration shooterSensor =
        new SensorConfiguration(new SensorConfiguration.IntegratedSensorSource(1));

    MotorConfiguration shooterConfig = new MotorConfiguration();
    shooterConfig.setIdleMode(IdleMode.COAST);
    shooterConfig.setPidProfile(Constants.Subsystem.Shooter.PID_PROFILE);
    shooterMotor = new SuMotor<SuSparkMax>(
        new SuSparkMax(new CANSparkMax(Constants.Motor.SHOOTER, MotorType.kBrushless), "Shooter"),
        shooterConfig, shooterSensor);

    // Setup shot interpolation distances.
    {
      shotSpeed = new Interpolator();
      //            m rpm
      shotSpeed.put(1, 3200);
      shotSpeed.put(1.5, 3600.0);
      shotSpeed.put(2, 3800);
      shotSpeed.put(3, 4200);
      shotSpeed.put(4, 4300);
    }
  }

  public void stop() {
    shooterMotor.stop();
  }

  /** 
   * Spool spools the shooter to the approximate speed for the specified distance in meters.
   */
  public void spool(double distance) {
    shooterMotor.set(ControlMode.VELOCITY, shotSpeed.interpolate(distance));
  }

  public void spoolDefault() {
    shooterMotor.set(ControlMode.VELOCITY, Constants.Subsystem.Shooter.DEFAULT_SPOOL_SPEED);
  }

  public double speed() {
    return shooterMotor.outputVelocity();
  }

  /**
   * Shoot attempts to make a shot by forcing the feeding of balls. It accepts a distance estimation in meters.
   */
  public void shoot(double distance) {
    double speed = shotSpeed.interpolate(distance);
    shooterMotor.set(ControlMode.VELOCITY, speed);

  }
}
