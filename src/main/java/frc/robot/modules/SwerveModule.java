package frc.robot.modules;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.sorutil.SorMath;

// SwerveModule manages an individual swerve drive module on the robot.
public class SwerveModule {

    // Should contain the two motor controllers, tracking, methods for setting,
    // and encoder output.
    private TalonFX powerController;
    private TalonSRX steerController;

    public SwerveModule(TalonFX powerController, TalonSRX steerController, int offSetTicks) {
        this.powerController = powerController;
        this.steerController = steerController;

        powerController.configFactoryDefault();
        steerController.configFactoryDefault();

        powerController.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        steerController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        steerController.setSensorPhase(true);
        steerController.configFeedbackNotContinuous(false, 50);
        steerController.setInverted(false);

        powerController.config_kF(0, 0);
        powerController.config_kP(0, 0.55);
        powerController.config_kI(0, 0);
        powerController.config_kD(0, 0.025);

        steerController.config_kF(0, 0);
        steerController.config_kP(0, 3.5);
        steerController.config_kI(0, 0);
        steerController.config_kD(0, 1.5);


        powerController.configPeakOutputForward(0.8);
        powerController.configPeakOutputReverse(-0.8);
        steerController.configPeakOutputForward(0.6);
        steerController.configPeakOutputReverse(-0.6);
       
        //steerController.setSelectedSensorPosition(steerController.getSensorCollection().getPulseWidthRiseToFallUs() - offSetTicks);
        steerController.getSensorCollection().syncQuadratureWithPulseWidth(0, 0, true, -offSetTicks + 4096, 50);
    }

    public double getSteerPosition() {
        return (SorMath.ticksToDegrees(steerController.getSelectedSensorPosition(), 4096));
    }

    public double getVelocity() {
        return powerController.getSelectedSensorVelocity();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(SorMath.sensorUnitsPer100msToMetersPerSecond(getVelocity()),
                Rotation2d.fromDegrees(getSteerPosition()));
    }

    public void setState(SwerveModuleState state) {
        if (state.speedMetersPerSecond * Constants.Subsystem.Swerve.Characteristics.COUNTS_PER_100MS < 400) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        powerController.set(ControlMode.Velocity, state.speedMetersPerSecond * Constants.Subsystem.Swerve.Characteristics.COUNTS_PER_100MS);
        steerController.set(ControlMode.Position, SorMath.degreesToTicks(state.angle.getDegrees(), 4096));
        
    }

    public void stop() {
        powerController.set(ControlMode.PercentOutput, 0);
    }

    public void periodic() {
        // Called at 50hz.
    }

}
