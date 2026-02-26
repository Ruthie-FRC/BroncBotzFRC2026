package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Setpoints;
import frc.robot.Constants;
import frc.robot.Constants.CanIDConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.TurretConstants.PivotConstants;

import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;
import static edu.wpi.first.units.Units.*;

public class HoodSubsystem extends SubsystemBase {
  private final SparkMax hoodMotor = new SparkMax(CanIDConstants.hoodID, MotorType.kBrushless);
  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(
              HoodConstants.kP,
              HoodConstants.kI,
              HoodConstants.kD,
              DegreesPerSecond.of(180),
              DegreesPerSecondPerSecond.of(90))
          .withSimClosedLoopController(
            HoodConstants.kPSim,
              HoodConstants.kISim,
              HoodConstants.kDSim,
              DegreesPerSecond.of(180),
              DegreesPerSecondPerSecond.of(90))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(1,1)))
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withVoltageCompensation(Volts.of(12))
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new SimpleMotorFeedforward(0, 0, 0, 1))
          .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController motor =
      new SparkWrapper(hoodMotor, DCMotor.getNEO(1), motorConfig);

 /*  private final MechanismPositionConfig robotToMechanism =
      new MechanismPositionConfig()
          .withMaxRobotHeight(Meters.of(1.5))
          .withMaxRobotLength(Meters.of(0.75))
          .withRelativePosition(new Translation3d(Meters.of(-0.25), // back from robot center
                                                  Meters.of(0),     // centered left/right
                                                  Meters.of(0.5))); // up from the floor reference*/

  private final PivotConfig m_config =
      new PivotConfig(motor)
          .withHardLimit(HoodConstants.hardLimitMin, HoodConstants.hardLimitMax)
          .withSoftLimits(HoodConstants.softLimitMin, HoodConstants.softLimitMax)
          .withTelemetry("TurretPivot", TelemetryVerbosity.HIGH)
          .withStartingPosition(Degrees.of(0))
          .withMOI(Meter.of(0.001), Pounds.of(3));

  private final Pivot hood = new Pivot(m_config);


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
