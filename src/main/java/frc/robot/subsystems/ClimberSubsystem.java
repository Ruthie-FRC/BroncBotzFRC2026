package frc.robot.subsystems;


import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Volts;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.system.plant.DCMotor;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

// TODO: Example with absolute encoders

/**
 * Exponentially profiled elevator subsystem. The elevator represented by this class does NOT have an absolute encoder! This
 * subsystem has a "self-homing" command, more details in the function description.
 */
public class ClimberSubsystem extends SubsystemBase
{
  /**
  * {@link SmartMotorControllerConfig} for the elevator motor.
  */
  private final SmartMotorControllerConfig      motorConfig    = new SmartMotorControllerConfig(this)
      /*
       * Basic Configuration options for the motor
       */
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Feedback Constants (PID Constants)
          .withClosedLoopController(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withSimClosedLoopController(ClimberConstants.kPSim, ClimberConstants.kISim, ClimberConstants.kDSim, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          // Feedforward Constants
          .withFeedforward(new ArmFeedforward(ClimberConstants.kS, ClimberConstants.kG, ClimberConstants.kV))
          .withSimFeedforward(new ArmFeedforward(ClimberConstants.kSSim, ClimberConstants.kGSim, ClimberConstants.kVSim))
          // Telemetry name and verbosity level
          .withTelemetry(ClimberConstants.motorTelemetryName, TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft.
          // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
          // You could also use .withGearing(12) which does the same thing.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(false)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withExternalEncoder(Constants.IntakeConstants.armMotor.getAbsoluteEncoder())
          .withExternalEncoderInverted(false)
          .withUseExternalFeedbackEncoder(true)
          .withExternalEncoderGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
          .withExternalEncoderZeroOffset(Degrees.of(0));
          ;

          private SparkMax spark = new SparkMax(4, MotorType.kBrushless);
          private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), motorConfig);
          private ArmConfig armCfg = new ArmConfig(sparkSmartMotorController)
            // Soft limit is applied to the SmartMotorControllers PID
            .withSoftLimits(Degrees.of(-20), Degrees.of(10))
            // Hard limit is applied to the simulation.
            .withHardLimit(Degrees.of(-30), Degrees.of(40))
            // Starting position is where your arm starts
            .withStartingPosition(Degrees.of(-5))
            // Length and mass of your arm for sim.
            .withLength(Feet.of(3))
            .withMass(Pounds.of(1))
            // Telemetry name and verbosity for the arm.
            .withTelemetry("Arm", TelemetryVerbosity.HIGH);

    // Arm Mechanism
    private Arm arm = new Arm(armCfg);

    /**
     * Set the angle of the arm.
     * @param angle Angle to go to.
     */
    public Command setAngle(Angle angle) { return arm.setAngle(angle);}
    /**
     * Move the arm up and down.
     * @param dutycycle [-1, 1] speed to set the arm too.
     */
    public Command set(double dutycycle) { return arm.set(dutycycle);}
    /**
     * Run sysId on the {@link Arm}
     */
    public Command sysId() { return arm.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));}
  /// Generic Smart Motor Controller with our options and vendor motor.

  public ClimberSubsystem()
  {

  }

  public void periodic()
  {
    arm.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    arm.simIterate();
  }

}
