// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.sorutil.motor.MotorConfiguration;
import frc.sorutil.motor.SensorConfiguration;
import frc.sorutil.motor.SuMotor;
import frc.sorutil.motor.SuVictorSpx;
import frc.sorutil.motor.SuMotor.IdleMode;

public class Storage extends SubsystemBase {
  private final Logger logger;

  private final SuMotor<SuVictorSpx> feederMotor;
  private final SuMotor<SuVictorSpx> storageMotor;

  private final DigitalInput intake, storage, feeder;
  private Double feederBallLocation = null;

  private StorageState storageState = StorageState.UNKNOWN;
  
  public static enum StorageState {
    UNKNOWN,
    CHECKING_FOR_BALL,
    EMPTY,
    BALL_AT_END,
    BALL_INSIDE,
    BALL_AT_END_AND_INSIDE,
    FEEDING_FEEDER,
    FEEDING_FEEDER_AND_BALL_INSIDE,
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
        new SensorConfiguration(new SensorConfiguration.ExternalSensorSource(sensor, 1));
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

  public StorageState state() {
    return storageState;
  }

  /**
   * While feed shooter is called, the storage will try to advance balls into the shooter.
   */
  public void feedShooter() {

  }

  /**
   * While manageStorage is called, the storage will attempt to automatically manage itself. While this is being called,
   * it can accept balls that are being picked up.
   */
  public void manageStorage() {

  }

  Timer stateTimer = new Timer();
  boolean lastEnabled = false;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (DriverStation.isEnabled()) {
      if (!lastEnabled) {
        stateTimer.start();
      }
    } else {
      if (lastEnabled) {
        stateTimer.stop();
        stateTimer.reset();
      }
    }

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
        break;

      case FEEDING_FEEDER:
        if (intakeSensor()) {
          nextState = StorageState.FEEDING_FEEDER_AND_BALL_INSIDE;
        }
        break;

      case BALL_AT_END_AND_INSIDE:
        break;

      case CHECKING_FOR_BALL:
        if (storageSensor()) {
          nextState = StorageState.BALL_AT_END;
        }
        if (stateTimer.get() > 3) {
          nextState = StorageState.EMPTY;
        }
        break;

      case FEEDING_FEEDER_AND_BALL_INSIDE:

        break;

      case EMPTY:
        if (intakeSensor()) {

        }
        break;
    }

    if (nextState == StorageState.BALL_AT_END) {
      storageMotor.stop();
    }

    if (storageState != nextState) {
      stateTimer.reset();
      stateTimer.start();
    }

    storageState = nextState;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
