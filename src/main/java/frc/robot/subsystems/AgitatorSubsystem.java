package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Agitator;
import frc.robot.Constants.CanIDConstants;
import frc.robot.Constants.IntakeRollerConstants;

import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class AgitatorSubsystem extends SubsystemBase {

  private final SparkMax agitator = new SparkMax(CanIDConstants.agitatorID, MotorType.kBrushless);

  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(
              Constants.Agitator.kP,
              Constants.Agitator.kI,
              Constants.Agitator.kD
              // RPM.of(5000),
              // RotationsPerSecondPerSecond.of(2500))
          )
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3)))
          .withIdleMode(MotorMode.COAST)
          .withTelemetry("FlywheelMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(
              new SimpleMotorFeedforward(
                  Constants.Agitator.kS, Constants.Agitator.kV, Constants.Agitator.kA))
          .withSimFeedforward(
              new SimpleMotorFeedforward(
                  Constants.Agitator.kS, Constants.Agitator.kV, Constants.Agitator.kA))
          .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController motor =
      new SparkWrapper(agitator, DCMotor.getNEO(1), motorConfig);

  private final FlyWheelConfig flywheelConfig =
      new FlyWheelConfig(motor)
          .withDiameter(Inches.of(4))
          .withMass(Pounds.of(1))
          .withTelemetry("FlywheelMech", TelemetryVerbosity.HIGH)
          .withSoftLimit(RPM.of(-5000), RPM.of(5000))
          .withSpeedometerSimulation(RPM.of(7500));

  private final FlyWheel flywheel = new FlyWheel(flywheelConfig);

  public AgitatorSubsystem() {}

  public AngularVelocity getVelocity() {
    return flywheel.getSpeed();
  }

  public Command setVelocity(AngularVelocity speed) {
    return flywheel.setSpeed(speed);
  }

  public Command setDutyCycle(double dutyCycle) {
    return flywheel.set(dutyCycle);
  }

  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return flywheel.setSpeed(speed);
  }

  public Command setDutyCycle(Supplier<Double> dutyCycle) {
    return flywheel.set(dutyCycle);
  }

  public Command sysId() {
    return flywheel.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));
  }

    public Command out() {
    return setDutyCycle(-Agitator.AgitatorRollerIntakeSpeeds);
  }

  public Command in() {
    return setDutyCycle(Agitator.AgitatorRollerIntakeSpeeds);
  }

  public Command stop() {
    return setDutyCycle(0);
  }

}
