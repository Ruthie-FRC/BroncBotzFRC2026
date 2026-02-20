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
import frc.robot.Constants.CanIDConstants;
import frc.robot.Constants.IndexerConstants;

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

public class IndexerSubsystem extends SubsystemBase {

  private final SparkMax flywheelMotor =
      new SparkMax(CanIDConstants.indexerflywheelID, MotorType.kBrushless);

  private final SparkMax kickerMotor = 
      new SparkMax(CanIDConstants.indexerflywheelID, MotorType.kBrushless);

  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(
              1, 0, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
          .withGearing(IndexerConstants.gearingIndexer)
          .withIdleMode(MotorMode.COAST)
          .withTelemetry("FlywheelMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withControlMode(ControlMode.CLOSED_LOOP);

    private final SmartMotorControllerConfig motorKickerConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(
              1, 0, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
          .withGearing(IndexerConstants.gearingKicker)
          .withIdleMode(MotorMode.COAST)
          .withTelemetry("FlywheelMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withControlMode(ControlMode.CLOSED_LOOP);


  private final SmartMotorController motor =
      new SparkWrapper(flywheelMotor, DCMotor.getNEO(1), motorConfig);

  private final SmartMotorController kickMotor =
      new SparkWrapper(kickerMotor, DCMotor.getNEO(1), motorKickerConfig);


  private final FlyWheelConfig flywheelConfig =
      new FlyWheelConfig(motor)
          .withDiameter(Inches.of(4))
          .withMass(Pounds.of(1))
          .withTelemetry("FlywheelMech", TelemetryVerbosity.HIGH)
          .withSoftLimit(RPM.of(-5000), RPM.of(5000))
          .withSpeedometerSimulation(RPM.of(7500));

  private final FlyWheel indexflywheel = new FlyWheel(flywheelConfig);


  private final FlyWheelConfig kickerConfig =
      new FlyWheelConfig(kickMotor)
          .withDiameter(Inches.of(4))
          .withMass(Pounds.of(1))
          .withTelemetry("FlywheelMech", TelemetryVerbosity.HIGH)
          .withSoftLimit(RPM.of(-5000), RPM.of(5000))
          .withSpeedometerSimulation(RPM.of(7500));

  private final FlyWheel kickerFlyWheel = new FlyWheel(flywheelConfig);

  public IndexerSubsystem() {}


  //INDEXER COMMANDS
  public AngularVelocity getVelocity() {
    return indexflywheel.getSpeed();
  }

  public Command setVelocity(AngularVelocity speed) {
    return indexflywheel.setSpeed(speed);
  }

  public Command setDutyCycleIndex(double dutyCycle) {
    return indexflywheel.set(dutyCycle);
  }

  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return indexflywheel.setSpeed(speed);
  }

  public Command setDutyCycle(Supplier<Double> dutyCycle) {
    return indexflywheel.set(dutyCycle);
  }

  public Command indexIn(){
    return setDutyCycleIndex(IndexerConstants.indexerSpeed);
  }

  public Command indexOut(){
    return setDutyCycleIndex(IndexerConstants.indexerSpeedOut);
  }

  //




  public Command sysId() {
    return indexflywheel.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));
  }
}
