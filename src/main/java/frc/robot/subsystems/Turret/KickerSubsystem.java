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

    private final SmartMotorControllerConfig motorKickerConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(
              1, 0, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
          .withGearing(IndexerConstants.gearingKicker)
          .withIdleMode(MotorMode.COAST)
          .withTelemetry("FlywheelMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(50))
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withControlMode(ControlMode.CLOSED_LOOP);

    private final SmartMotorController kickMotor =
      new SparkWrapper(kickerMotor, DCMotor.getNEO(1), motorKickerConfig);


    private final FlyWheelConfig kickerConfig =
    new FlyWheelConfig(kickMotor)
        .withDiameter(Inches.of(4))
        .withMass(Pounds.of(1))
        .withTelemetry("FlywheelMech", TelemetryVerbosity.HIGH)
        .withSoftLimit(RPM.of(-5000), RPM.of(5000))
        .withSpeedometerSimulation(RPM.of(7500));

    private final FlyWheel kickerFlyWheel = new FlyWheel(kickerConfig);

    public Command setKickerVolts(double volts){
      return kickerFlyWheel.setVoltage(Volts.of(volts));
    }

    public Command kickerShoot(){
      return setKickerVolts(KickerConstants.kickerVoltage);
    }

    public Command kickerUnshoot(){
      return setKickerVolts(KickerConstants.kickerVoltageReverse);
    }
}
