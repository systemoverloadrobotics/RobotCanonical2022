package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.sorutil.motor.MotorConfiguration;
import frc.sorutil.motor.SuMotor;
import frc.sorutil.motor.SuTalonFx;
import frc.sorutil.motor.SuMotor.ControlMode;
import frc.sorutil.motor.SuMotor.IdleMode;

public class Intake extends SubsystemBase {
  private final DoubleSolenoid intakeExtension;
  private final SuMotor<SuTalonFx> intake;

  public Intake() {
    intakeExtension = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
        Constants.Solenoid.INTAKE_EXTEND, Constants.Solenoid.INTAKE_RETRACT);

    MotorConfiguration intakeConfig = new MotorConfiguration();
    intakeConfig.setIdleMode(IdleMode.COAST);
    intakeConfig.setPidProfile(Constants.Subsystem.Intake.PID_PROFILE);
    intake =
        new SuMotor<SuTalonFx>(new SuTalonFx(new WPI_TalonFX(Constants.Motor.INTAKE), "Intake"), intakeConfig);
  }

  public void retract() {
    intakeExtension.set(Value.kReverse);
  }

  public void intake() {
    intakeExtension.set(Value.kForward);

    intake.set(ControlMode.VELOCITY, Constants.Subsystem.Intake.SPEED);
  }

  public void reverse() {
    intakeExtension.set(Value.kForward);

    intake.set(ControlMode.PERCENT_OUTPUT, -1);
  }
}
