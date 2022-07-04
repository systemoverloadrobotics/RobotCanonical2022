package frc.robot.subsystems;

import java.util.logging.Level;
import java.util.logging.Logger;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
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

public class Storage extends SubsystemBase {
  private final Logger logger;

  private final SuMotor<SuVictorSpx> feederMotor;
  private final SuMotor<SuVictorSpx> storageMotor;

  private final DigitalInput intake, storage, feeder;

  private StorageState storageState = StorageState.UNKNOWN;
  private FeederState feederState = FeederState.UNKNOWN;

  public static enum StorageState {
    UNKNOWN, CHECKING_FOR_BALL, EMPTY, BALL_AT_END, BALL_INSIDE, BALL_AT_END_AND_INSIDE, FEEDING_FEEDER, FEEDING_FEEDER_AND_INSIDE,
  }

  public static enum FeederState {
    UNKNOWN, CHECKING_FOR_BALL, EMPTY, ACCEPTING_BALL, HOLDING_BALL, FEEDING,
  }

  /** Creates a new Storage. */
  public Storage() {
    logger = Logger.getLogger(ExampleSubsystem.class.getName());

    intake = new DigitalInput(Constants.Sensors.INTAKE_BREAKBEAM);
    storage = new DigitalInput(Constants.Sensors.STORAGE_BREAKBEAM);
    feeder = new DigitalInput(Constants.Sensors.FEEDER_BREAKBEAM);

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
    feederMotor =
        new SuMotor<>(new SuVictorSpx(new WPI_VictorSPX(Constants.Motor.FEEDER), "Feeder"),
            feederConfig, feederSensor);

    MotorConfiguration storageConfig = new MotorConfiguration();
    storageConfig.setIdleMode(IdleMode.BRAKE);
    storageMotor = new SuMotor<>(
        new SuVictorSpx(new WPI_VictorSPX(Constants.Motor.STORAGE), "Storage"), storageConfig);
  }

  public boolean intakeSensor() {
    return intake.get();
  }

  public boolean storageSensor() {
    return storage.get();
  }

  public boolean feederSensor() {
    return feeder.get();
  }

  public StorageState storageState() {
    return storageState;
  }
  
  public FeederState feederState() {
    return feederState;
  }

  // Defines if either of the feeding or management methods were called this tick.
  private boolean feedRequested, manageRequested, reverseRequested;

  /**
   * While feed shooter is called, the storage will try to advance balls into the shooter.
   */
  public void feedShooter() {
    feedRequested = true;
  }

  /**
   * While manageStorage is called, the storage will attempt to automatically manage itself. While this is being called,
   * it can accept balls that are being picked up.
   */
  public void manageStorage() {
    manageRequested = true;
  }

  /**
   * While reverse is called, the storage will run full speed in reverse in an attempt to clear the system.
   */
  public void reverse() {
    reverseRequested = true;
  }

  Timer storageStateTimer = new Timer();
  Timer feederStateTimer = new Timer();
  boolean lastEnabled = false;

  private StorageState computeNextStorageState() {
    StorageState nextState = storageState;
    switch (storageState) {
      case UNKNOWN:
        nextState = StorageState.CHECKING_FOR_BALL;
        if (storageSensor()) {
          nextState = StorageState.BALL_AT_END;
        }
        if (intakeSensor()) {
          nextState = StorageState.BALL_INSIDE;
        }
        break;

      case BALL_INSIDE:
        if (storageSensor()) {
          nextState = StorageState.BALL_AT_END;
        }
        break;

      case BALL_AT_END:
        if (intakeSensor()) {
          nextState = StorageState.BALL_AT_END_AND_INSIDE;
        }
        // fallthrough
      case BALL_AT_END_AND_INSIDE:
        if (feederState == FeederState.EMPTY) {
          feederState = FeederState.ACCEPTING_BALL;
          if (storageState == StorageState.BALL_AT_END) {
            nextState = StorageState.FEEDING_FEEDER;
          } else {
            nextState = StorageState.FEEDING_FEEDER_AND_INSIDE;
          }
        }
        break;

      case CHECKING_FOR_BALL:
        if (storageSensor()) {
          nextState = StorageState.BALL_AT_END;
        }
        if (storageStateTimer.get() > 3) {
          nextState = StorageState.EMPTY;
        }
        break;

      case EMPTY:
        if (intakeSensor()) {
          nextState = StorageState.BALL_INSIDE;
        }
        break;
      case FEEDING_FEEDER:
        if (intakeSensor()) {
          nextState = StorageState.FEEDING_FEEDER_AND_INSIDE;
        }
        break;
      case FEEDING_FEEDER_AND_INSIDE:
        break;
    }

    return nextState;
  }

  private FeederState computeNextFeederState() {
    FeederState nextState = feederState;
    switch (feederState) {
      case UNKNOWN:
        nextState = FeederState.CHECKING_FOR_BALL;
        // fallthrough
      case CHECKING_FOR_BALL:
        if (feederStateTimer.get() > 2) {
          nextState = FeederState.EMPTY;
        }
        // fallthrough
      case ACCEPTING_BALL:
        if (feederSensor()) {
          nextState = FeederState.HOLDING_BALL;
          if (storageState == StorageState.FEEDING_FEEDER) {
            storageState = StorageState.EMPTY;
          }
          if (storageState == StorageState.FEEDING_FEEDER_AND_INSIDE) {
            storageState = StorageState.BALL_INSIDE;
          }
          // Note this causes the first ball to be misaligned if the ball is found through CHECKING_FOR_BALL.
          feederMotor.setSensorPosition(0);
        }
        break;
      case HOLDING_BALL:
        if (feedRequested) {
          nextState = FeederState.FEEDING;
        }
        break;
      case FEEDING:
        if (!feedRequested) {
          nextState = FeederState.HOLDING_BALL;
        }
        // If the feeder has rotated past the estimated distance required to fire.
        if (feederMotor.outputPosition() > Constants.Subsystem.Shooter.FEEDER_TRIGGER_POSITION) {
          nextState = FeederState.EMPTY;
        }
        break;
      case EMPTY:
        break;
    }
    return nextState;
  }

  boolean ballExitTriggered = false;

  /**
   * ballExited should be called once a tick to determine if a ball has left the front of the robot.
   * 
   * @return true on exactly one tick where the ball leaving has been detected.
   */
  private boolean ballExited() {
    if (reverseRequested) {
      if (!ballExitTriggered && intakeSensor()) {
        ballExitTriggered = true;
      }

      if (ballExitTriggered && !intakeSensor()) {
        ballExitTriggered = false;
        return true;
      }
    } else {
      ballExitTriggered = false;
    }

    return false;
  }

  private StorageState computeNextStorageStateReverse(boolean ballExited) {
    StorageState nextState = storageState;

    switch (storageState) {
      case BALL_AT_END:
        if (!storageSensor()) {
          nextState = StorageState.BALL_INSIDE;
        }
        break;
      case BALL_AT_END_AND_INSIDE:
        if (ballExited) {
          nextState = StorageState.BALL_INSIDE;
        }
        break;
      case BALL_INSIDE:
        if (ballExited) {
          nextState = StorageState.EMPTY;
        }
        break;
      case CHECKING_FOR_BALL:
        if (ballExited) {
          nextState = StorageState.EMPTY;
        }
        break;
      case FEEDING_FEEDER:
        if (storageSensor()) {
          nextState = StorageState.BALL_AT_END;
        }
        break;
      case FEEDING_FEEDER_AND_INSIDE:
        if (storageSensor()) {
          nextState = StorageState.BALL_AT_END_AND_INSIDE;
        }
        if (ballExited) {
          nextState = StorageState.BALL_INSIDE;
        }
        break;
      case UNKNOWN:
        break;
      case EMPTY:
        break;
    }

    return nextState;
  }

  private FeederState computeNextFeederStateReverse() {
    FeederState nextState = feederState;

    switch (feederState) {
      case FEEDING:
        // fallthrough
      case ACCEPTING_BALL:
        // fallthrough
      case HOLDING_BALL:
        if (!(storageState == StorageState.BALL_AT_END
            || storageState == StorageState.BALL_AT_END_AND_INSIDE) && storageSensor()) {
          nextState = FeederState.EMPTY;
        }
        break;
      case EMPTY:
        // fallthrough
      case CHECKING_FOR_BALL:
        // fallthrough
      case UNKNOWN:
        break;
    }

    return nextState;
  }

  private void updateTimers(boolean shouldResetStorage, boolean shouldResetFeeder) {
    if (DriverStation.isEnabled()) {
      if (!lastEnabled) {
        storageStateTimer.start();
        feederStateTimer.start();
      }
    } else {
      if (lastEnabled) {
        storageStateTimer.stop();
        storageStateTimer.reset();

        feederStateTimer.stop();
        feederStateTimer.reset();
      }
    }

    if (shouldResetStorage) {
      storageStateTimer.reset();
      storageStateTimer.start();
    }
    if (shouldResetFeeder) {
      feederStateTimer.reset();
      feederStateTimer.start();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    StorageState nextStorageState = computeNextStorageState();
    FeederState nextFeederState = computeNextFeederState();

    // compute whether a ball has exited the front just in case.
    boolean ballExited = ballExited();

    if (!reverseRequested) {
      // Update motors only if the management or feed command is running.
      if (manageRequested || feedRequested) {
        // Update the motors based on the states
        if (nextStorageState == StorageState.BALL_INSIDE
            || nextStorageState == StorageState.FEEDING_FEEDER_AND_INSIDE
            || nextStorageState == StorageState.FEEDING_FEEDER
            || nextStorageState == StorageState.CHECKING_FOR_BALL) {
          storageMotor.set(ControlMode.PERCENT_OUTPUT, Constants.Subsystem.STORAGE_SPEED);
        }
        if (nextStorageState == StorageState.BALL_AT_END
            || nextStorageState == StorageState.BALL_AT_END_AND_INSIDE) {
          storageMotor.stop();
        }

        if (nextFeederState == FeederState.ACCEPTING_BALL) {
          feederMotor.set(ControlMode.VELOCITY, Constants.Subsystem.Shooter.FEEDER_FIND_SPEED);
        }

        if (nextFeederState == FeederState.CHECKING_FOR_BALL) {
          feederMotor.set(ControlMode.VELOCITY, -Constants.Subsystem.Shooter.FEEDER_FIND_SPEED);
        }

        if (nextFeederState == FeederState.HOLDING_BALL) {
          feederMotor.set(ControlMode.POSITION, Constants.Subsystem.Shooter.FEEDER_HOLD_POSITION);
        }

        if (nextFeederState == FeederState.FEEDING) {
          // Set the feeder motor to a position past 1 full rotation, effectively over-feeding the shooter.
          feederMotor.set(ControlMode.POSITION, Constants.Subsystem.Shooter.FEEDER_FIRE_POSITION);
        }
      } else {
        feederMotor.stop();
        storageMotor.stop();
      }
    } else {
      feederMotor.set(ControlMode.PERCENT_OUTPUT, -1);
      storageMotor.set(ControlMode.PERCENT_OUTPUT, -1);

      nextStorageState = computeNextStorageStateReverse(ballExited);
      nextFeederState = computeNextFeederStateReverse();
    }

    {
      // Logging operations
      String currentCommand = null;
      if (manageRequested) {
        currentCommand = "Manage";
      } else if (reverseRequested) {
        currentCommand = "Reverse";
      } else if (feedRequested) {
        currentCommand = "Feed";
      }

      if (feederState != nextFeederState) {
        logger.log(Level.INFO, String.format(
            "Updating to next feeder state: (current command: %s) Last state: %s, Next state: %s",
            currentCommand, feederState.name(), nextFeederState.name()));
      }

      if (storageState != nextStorageState) {
        logger.log(Level.INFO, String.format(
            "Updating to next feeder state: (current command: %s) Last state: %s, Next state: %s",
            currentCommand, storageState.name(), nextStorageState.name()));
      }
    }

    { 
      // Apply per-tick updates
      updateTimers(storageState != nextStorageState, feederState != nextFeederState);
      // Update state only if a command is being invoked, otherwise remain stationary.
      if (feedRequested || reverseRequested || manageRequested) {
        storageState = nextStorageState;
        feederState = nextFeederState;
      }
      manageRequested = false;
      reverseRequested = false;
      feedRequested = false;
    }
  }
}
