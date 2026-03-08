package frc.robot.subsystems.Turret;

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
import frc.robot.Constants.TurretConstants.IndexerConstants;

import java.util.function.Supplier;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IndexerSubsystem extends SubsystemBase {

  private final SparkMax indexMotor =
      new SparkMax(CanIDConstants.indexerflywheelID, MotorType.kBrushless);

  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(
              0, 0, 0)
          .withGearing(IndexerConstants.gearingIndexer)
          .withIdleMode(MotorMode.COAST)
          .withTelemetry("Indexer", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController motor =
      new SparkWrapper(indexMotor, DCMotor.getNEO(1), motorConfig);

  private final FlyWheelConfig flywheelConfig =
      new FlyWheelConfig(motor)
          .withDiameter(Inches.of(4))
          .withMass(Pounds.of(1))
          .withTelemetry("FlywheelMech", TelemetryVerbosity.HIGH)
          .withSoftLimit(RPM.of(-5000), RPM.of(5000))
          .withSpeedometerSimulation(RPM.of(7500));

  private final FlyWheel indexflywheel = new FlyWheel(flywheelConfig);

  public IndexerSubsystem() {}


  //INDEXER COMMANDS
  public Command setDutyCycle(double dutyCycle) {
    return indexflywheel.set(dutyCycle);
  }
  
  public Command setRPM(double rpm){
     return indexflywheel.setSpeed(RPM.of(rpm));
  }

  public Command stop(){
    return indexflywheel.set(0);
  }

  public AngularVelocity getRPM(){
    return indexflywheel.getSpeed();
  }

  public void periodic()
  {
    indexflywheel.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    indexflywheel.simIterate();
  }
}
