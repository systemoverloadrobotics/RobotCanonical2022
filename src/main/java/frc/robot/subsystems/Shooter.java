package frc.robot.subsystems;

import java.util.logging.Logger;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import frc.robot.manager.StorageManager;
import frc.sorutil.SorMath;
import frc.sorutil.interpolate.Interpolator;
import frc.sorutil.motor.MotorConfiguration;
import frc.sorutil.motor.SensorConfiguration;
import frc.sorutil.motor.SuMotor;
import frc.sorutil.motor.SuSparkMax;
import frc.sorutil.motor.SuTalonSrx;
import frc.sorutil.motor.SuMotor.ControlMode;
import frc.sorutil.motor.SuMotor.IdleMode;

public class Shooter {
  private final SuMotor<SuSparkMax> shooterMotor;
  private final SuMotor<SuTalonSrx> feederMotor;

  private final Logger logger;
  private final StorageManager storage;

  private final Interpolator shotSpeed;

  public Shooter(StorageManager storage) {
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

    Encoder feederEncoder = new Encoder(Constants.Subsystem.Shooter.FEEDER_ENCODER_CHANNEL_A,
        Constants.Subsystem.Shooter.FEEDER_ENCODER_CHANNEL_B);
    SensorConfiguration.ExternalSensor sensor =
        new SensorConfiguration.Encoder(feederEncoder, 2048);
    SensorConfiguration feederSensor =
        new SensorConfiguration(new SensorConfiguration.ExternalSensorSource(sensor, 1));
    MotorConfiguration feederConfig = new MotorConfiguration();
    feederConfig.setIdleMode(IdleMode.BRAKE);
    feederConfig.setPidProfile(Constants.Subsystem.Shooter.FEEDER_PROFILE);
    feederMotor =
        new SuMotor<SuTalonSrx>(new SuTalonSrx(new WPI_TalonSRX(Constants.Motor.FEEDER), "Feeder"),
            feederConfig, feederSensor);
    this.storage = storage;

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
    feederMotor.stop();
  }

  /** 
   * Spool spools the shooter to the approximate speed for the specified distance in meters.
   */
  public void spool(double distance) {
    shooterMotor.set(ControlMode.VELOCITY, shotSpeed.interpolate(distance));
  }

  /**
   * Shoot attempts to make a shot by forcing the feeding of balls. It accepts a distance estimation in meters.
   */
  public void shoot(double distance) {
    double speed = shotSpeed.interpolate(distance);
    shooterMotor.set(ControlMode.VELOCITY, speed);

    if (SorMath.epsilonEquals(shooterMotor.outputVelocity(), speed, speed * Constants.Subsystem.Shooter.SHOOTER_ERROR_TOLERANCE)) {
      feederMotor.set(ControlMode.VELOCITY, Constants.Subsystem.Shooter.FEEDER_SPEED);
    }
  }

  public void resetFeeder() {
    feederMotor.setSensorPosition(0);
  }

  public void setFeederPosition(double degrees) {
    feederMotor.set(ControlMode.POSITION, degrees);
  }
}
