package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
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
import frc.robot.Constants.TurretConstants.KickerConstants;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class KickerSubsystem extends SubsystemBase{
    private final SparkMax kickerMotor = 
      new SparkMax(CanIDConstants.kickerflywheelID, MotorType.kBrushless);

    private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(
              0, 0, 0)
          .withGearing(IndexerConstants.gearingKicker)
          .withIdleMode(MotorMode.COAST)
          .withTelemetry("KickerMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(60))
          .withMotorInverted(true)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withControlMode(ControlMode.CLOSED_LOOP);

    private final SmartMotorController motor =
      new SparkWrapper(kickerMotor, DCMotor.getNEO(1), motorConfig);


    private final FlyWheelConfig kickerConfig =
    new FlyWheelConfig(motor)
        .withDiameter(Inches.of(2))
        .withMass(Pounds.of(0.7))
        .withTelemetry("Kicker", TelemetryVerbosity.HIGH)
        .withSoftLimit(RPM.of(-5000), RPM.of(5000))
        .withSpeedometerSimulation(RPM.of(7500));

    private final FlyWheel kicker = new FlyWheel(kickerConfig);

    public Command setDutyCycle(double dutyCycle) {
      return kicker.set(dutyCycle);
    }
    
    public Command setRPM(double rpm){
       return kicker.setSpeed(RPM.of(rpm));
    }

    public Command stop(){
      return kicker.set(0);
    }

    public void periodic()
    {
      kicker.updateTelemetry();
    }
  
    public void simulationPeriodic()
    {
      kicker.simIterate();
    }

    public AngularVelocity getRPM(){
      return kicker.getSpeed();
    }
}
