package frc.robot.modules;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.sorutil.SorMath;
import frc.sorutil.motor.MotorConfiguration;
import frc.sorutil.motor.SensorConfiguration;
import frc.sorutil.motor.SuMotor;
import frc.sorutil.motor.SuTalonFx;
import frc.sorutil.motor.SuTalonSrx;
import frc.sorutil.motor.SensorConfiguration.ConnectedSensorSource;
import frc.sorutil.motor.SensorConfiguration.ConnectedSensorType;
import frc.sorutil.motor.SensorConfiguration.IntegratedSensorSource;
import frc.sorutil.motor.SuMotor.ControlMode;

// SwerveModule manages an individual swerve drive module on the robot.
public class SwerveModule {

  // Should contain the two motor controllers, tracking, methods for setting,
  // and encoder output.
  private TalonFX powerController;
  private TalonSRX steerController;

  private SuMotor<SuTalonFx> powerController2;
  private SuMotor<SuTalonSrx> steerController2;

  public SwerveModule(String name, int powerIdx, int steerIdx, int offSetTicks) {
    MotorConfiguration powerConfig = new MotorConfiguration();
    powerConfig.setPidProfile(Constants.Subsystem.Swerve.POWER_PROFILE);
    powerConfig.setMaxOutput(0.8);
    SensorConfiguration powerSensorConfig = new SensorConfiguration(new IntegratedSensorSource(6.55));
    powerController2 = new SuMotor<>(new SuTalonFx(new WPI_TalonFX(powerIdx), name+"-power"), powerConfig, powerSensorConfig);

    MotorConfiguration steerConfig = new MotorConfiguration();
    steerConfig.setMaxOutput(0.6);
    steerConfig.setPidProfile(Constants.Subsystem.Swerve.STEER_PROFILE);
    ConnectedSensorSource steerSource = new ConnectedSensorSource(4096, 1, ConnectedSensorType.MAG_ENCODER_RELATIVE);
    SensorConfiguration steerSensorConfig = new SensorConfiguration(steerSource);
    steerController2 = new SuMotor<>(new SuTalonSrx(new WPI_TalonSRX(steerIdx), name+"-steer"), steerConfig, steerSensorConfig);

    ((WPI_TalonSRX)steerController2.rawController()).getSensorCollection().syncQuadratureWithPulseWidth(0, 0, true,
        -offSetTicks + 4096, 50);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(powerController2.outputVelocity(),
        Rotation2d.fromDegrees(steerController2.outputPosition()));
  }

  public void setState(SwerveModuleState state) {
    if (state.speedMetersPerSecond
        * Constants.Subsystem.Swerve.Characteristics.COUNTS_PER_100MS < 400) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    powerController2.set(ControlMode.VELOCITY,
        state.speedMetersPerSecond * Constants.Subsystem.Swerve.Characteristics.MPS_TO_RPM);
    steerController2.set(ControlMode.POSITION, state.angle.getDegrees());

  }

  public void stop() {
    powerController2.set(ControlMode.PERCENT_OUTPUT, 0);
  }
}
