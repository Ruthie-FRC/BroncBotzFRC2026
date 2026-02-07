package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.MomentOfInertiaUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDConstants;
import frc.robot.Constants.HoodConstants;

import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

import static edu.wpi.first.units.Units.*;

public class HoodSubsystem extends SubsystemBase {
  private final TalonFX hoodMotor = new TalonFX(CanIDConstants.hoodID);

  private final SmartMotorControllerConfig hoodMotorConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(
              0.00016541, 0, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(24,54)))
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
          .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withMomentOfInertia(Inches.of(4),Pound.of(4));

  private final SmartMotorController hoodSMC =
      new TalonFXWrapper(hoodMotor, DCMotor.getKrakenX60(1), hoodMotorConfig);//Change: field oriented control?

  private final ArmConfig hoodConfig =
      new ArmConfig(hoodSMC)
          .withTelemetry("HoodMech", TelemetryVerbosity.HIGH)
          .withSoftLimits(HoodConstants.softLimitMin, HoodConstants.softLimitMax)//soft limit?
          .withHardLimit(HoodConstants.hardLimitMin, HoodConstants.hardLimitMax)
          .withLength(HoodConstants.length)
          .withStartingPosition(HoodConstants.hardLimitMin)
          .withMOI(HoodConstants.MOIInKilogram);

  private final Arm hood = new Arm(hoodConfig);

  public HoodSubsystem() {}

  public Command setAngle(Angle angle) {
    return hood.setAngle(angle);
  }

  public Command setAngle(Supplier<Angle> angleSupplier) {
    return hood.setAngle(angleSupplier);
  }

  public Angle getAngle() {
    return hood.getAngle();
  }

  public Command sysId() {
    return hood.sysId(
        Volts.of(4.0), // maximumVoltage
        Volts.per(Second).of(0.5), // step
        Seconds.of(8.0) // duration
        );
  }

  public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
    return hood.set(dutyCycleSupplier);
  }

  public Command setDutyCycle(double dutyCycle) {
    return hood.set(dutyCycle);
  }

  @Override
  public void periodic() {
    hood.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    hood.simIterate();
  }
}
