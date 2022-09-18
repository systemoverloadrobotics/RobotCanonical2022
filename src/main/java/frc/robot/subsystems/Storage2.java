package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.sorutil.motor.MotorConfiguration;
import frc.sorutil.motor.SensorConfiguration;
import frc.sorutil.motor.SuMotor;
import frc.sorutil.motor.SuVictorSpx;
import frc.sorutil.motor.SuMotor.ControlMode;
import frc.sorutil.motor.SuMotor.IdleMode;

public class Storage2 extends SubsystemBase {
  private static enum StorageState {
    UNKNOWN, STANDBY, REVERSING, CHECKING_FOR_BALL, BALL_READY,
  }

  private static enum FeederState {
    UNKNOWN, STANDBY, REVERSING, ACCEPTING_BALL, HOLDING_BALL, FEEDING,
  }
  // Define motors
  private final SuMotor<SuVictorSpx> feederMotor;
  private final SuMotor<SuVictorSpx> storageMotor;

  // Define sensors
  private final DigitalInput intake, storage, feeder;

  // Set initial values for state machines
  private StorageState storageState = StorageState.UNKNOWN;
  private FeederState feederState = FeederState.UNKNOWN;

  // SetState methods used to centralize logging in the future
  private void setStorageState(StorageState newState) {
    storageState = newState;
  }

  private void setFeederState(FeederState newState) {
    feederState = newState;
  }

  // Define stopwatches used in state machines
  private Timer storageStateTimer = new Timer();
  private Timer feederStateTimer = new Timer();

  private void resetStoreageTimer() {
    storageStateTimer.stop();
    storageStateTimer.reset();
  }

  private void resetFeederTimer() {
    feederStateTimer.stop();
    feederStateTimer.reset();
  }

  public Storage2() {

    // Digital Sensor Setup
    intake = new DigitalInput(Constants.Sensors.INTAKE_BREAKBEAM);
    storage = new DigitalInput(Constants.Sensors.STORAGE_BREAKBEAM);
    feeder = new DigitalInput(Constants.Sensors.FEEDER_BREAKBEAM);

    // Motor initalization
    feederMotor = feederMotorInit();
    storageMotor = storageMotorInit();

  }

  private SuMotor<SuVictorSpx> storageMotorInit() {
    MotorConfiguration storageConfig = new MotorConfiguration();
    storageConfig.setIdleMode(IdleMode.BRAKE);

    return new SuMotor<>(new SuVictorSpx(new WPI_VictorSPX(Constants.Motor.STORAGE), "Storage"),
        storageConfig);
  }

  private SuMotor<SuVictorSpx> feederMotorInit() {
    Encoder feederEncoder = new Encoder(Constants.Subsystem.Shooter.FEEDER_ENCODER_CHANNEL_A,
        Constants.Subsystem.Shooter.FEEDER_ENCODER_CHANNEL_B);

    SensorConfiguration.ExternalSensor sensor =
        new SensorConfiguration.Encoder(feederEncoder, 2048);

    SensorConfiguration feederSensor =
        new SensorConfiguration(new SensorConfiguration.ExternalSensorSource(sensor,
            Constants.Subsystem.Shooter.FEEDER_GEARBOX_RATIO));

    MotorConfiguration feederConfig = new MotorConfiguration();
    feederConfig.setIdleMode(IdleMode.BRAKE);
    feederConfig.setPidProfile(Constants.Subsystem.Shooter.FEEDER_PROFILE);

    return new SuMotor<>(new SuVictorSpx(new WPI_VictorSPX(Constants.Motor.FEEDER), "Feeder"),
        feederConfig, feederSensor);
  }

  private void evaluateStorageState() {

    if (reverseRequested) {
      setStorageState(StorageState.REVERSING);
      storageMotor.set(ControlMode.PERCENT_OUTPUT, -1);
    } else {

      switch (storageState) {
        case STANDBY:
          storageMotor.stop();
          if (manageRequested || feedRequested) {
            setStorageState(StorageState.CHECKING_FOR_BALL);
            storageStateTimer.start();
          }
          break;

        case CHECKING_FOR_BALL:
          storageMotor.set(ControlMode.PERCENT_OUTPUT, Constants.Subsystem.STORAGE_SPEED);
          if (storageStateTimer.get() > 3) {
            setStorageState(StorageState.STANDBY);
          } else if (storage.get() && intake.get()) {
            setStorageState(StorageState.BALL_READY);
          }
          break;

        case BALL_READY:
          storageMotor.stop();
          if (!storage.get()) {
            setStorageState(StorageState.STANDBY);
          }
          break;

        default:
          setStorageState(StorageState.STANDBY);
          resetStoreageTimer();
          break;
      }
    }
  }

  private void evaluateFeederState() {
    if (reverseRequested) {
      setFeederState(FeederState.REVERSING);
      feederMotor.set(ControlMode.PERCENT_OUTPUT, -1);
    } else {
      switch (feederState) {
        case STANDBY:
          feederMotor.stop();

          if (storageState == StorageState.BALL_READY && (manageRequested || feedRequested)) {
            setFeederState(FeederState.ACCEPTING_BALL);
            resetFeederTimer();
            feederStateTimer.start();
          }
          break;

        case ACCEPTING_BALL:
          if (feederStateTimer.get() > 2) {
            setFeederState(FeederState.STANDBY);
          } else if (feeder.get() && (manageRequested || feedRequested)) {
            setFeederState(FeederState.HOLDING_BALL);
          }
          break;

        case HOLDING_BALL:
          feederMotor.stop();
          if (feedRequested) {
            setFeederState(FeederState.FEEDING);
          }
          break;

        case FEEDING:
          feederMotor.set(ControlMode.VELOCITY, Constants.Subsystem.Shooter.FEEDER_FIND_SPEED);

          if (!feedRequested) {
            setFeederState(FeederState.HOLDING_BALL);
          } else if (feederMotor
              .outputPosition() > Constants.Subsystem.Shooter.FEEDER_TRIGGER_POSITION) {
            setFeederState(FeederState.STANDBY);
          }
          break;

        default:
          setFeederState(FeederState.STANDBY);
          break;
      }
    }
  }

  // Defines if either of the feeding or management methods were called this tick.
  private boolean feedRequested, manageRequested, reverseRequested;

  @Override
  public void periodic() {
    evaluateStorageState();
    evaluateFeederState();
    resetLocalFlags();
  }

  private void resetLocalFlags() {
    manageRequested = false;
    reverseRequested = false;
    feedRequested = false;
  }

  /**
   * While feed shooter is called, the storage will try to advance balls into the shooter.
   */
  public void feedShooter() {
    feedRequested = true;
  }

  /**
   * While manage is called, the storage will attempt to automatically manage itself. While this is being called, it can
   * accept balls that are being picked up.
   */
  public void manage() {
    manageRequested = true;
  }

  /**
   * While reverse is called, the storage will run full speed in reverse in an attempt to clear the system.
   */
  public void reverse() {
    reverseRequested = true;
  }
}
