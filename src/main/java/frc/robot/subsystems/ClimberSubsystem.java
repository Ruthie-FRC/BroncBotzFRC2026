package frc.robot.subsystems;


import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Climber;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
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

  /*
   * This is the STARTING PID Controller for the Elevator. If you are using a TalonFX or TalonFXS this will run on the motor controller itself.
   */
  private final ExponentialProfilePIDController pidController  = new ExponentialProfilePIDController(1,
                                                                                                     0,
                                                                                                     0,
                                                                                                     ExponentialProfilePIDController.createElevatorConstraints(
                                                                                                           Volts.of(12),
                                                                                                           Climber.dcMotor,
                                                                                                           Climber.weight,
                                                                                                           Climber.radius,
                                                                                                           Climber.gearing));
  /*Climber.
   * This is the STARTING Feedforward for the Elevator. If you are using a TalonFX or TalonFXS this will run on the motor controller itself.
   */
  private final ElevatorFeedforward             elevatorFeedforward = new ElevatorFeedforward(0, 0, 0, 0);
  /**
  * {@link SmartMotorControllerConfig} for the elevator motor.
  */
  private final SmartMotorControllerConfig      motorConfig    = new SmartMotorControllerConfig(this)
      /*
       * Basic Configuration options for the motor
       */
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withMechanismCircumference(Climber.circumference)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withStatorCurrentLimit(Amps.of(40)) // Prevents our motor from continuously over-taxing itself when it is stuck.
      .withClosedLoopRampRate(Seconds.of(0.25)) // Prevents our motor from rapid demand changes that could cause dramatic voltage drops, and current draw.
      .withOpenLoopRampRate(Seconds.of(0.25)) // Same as above
      .withTelemetry(Climber.motorTelemetryName,
                     TelemetryVerbosity.HIGH) // Could have more fine-grained control over what gets reported with SmartMotorControllerTelemetryConfig
      /*
       * Closed loop configuration options for the motor.
       */
      .withClosedLoopController(pidController)
      .withFeedforward(elevatorFeedforward)
      .withSoftLimit(Climber.softLowerLimit, Climber.softUpperLimit);
  /// Generic Smart Motor Controller with our options and vendor motor.
  private final SmartMotorController motor         = new SparkWrapper(Climber.elevatorMotor, Climber.dcMotor, motorConfig);
  /// Elevator-specific options
  private       ElevatorConfig       m_config      = new ElevatorConfig(motor)
      /*
       * Basic configuration options for the arm.
       */
      .withMass(Climber.weight)
      .withStartingHeight(Climber.startingHeight) // The starting position should ONLY be defined if you are NOT using an absolute encoder.
      .withTelemetry(Climber.mechTelemetryName, TelemetryVerbosity.HIGH)
      /*
       * Simulation configuration options for the arm.
       */
      .withHardLimits(Climber.hardLowerLimit, Climber.hardUpperLimit);
  // Arm mechanism
  private final Elevator             m_elevator    = new Elevator(m_config);

  public ClimberSubsystem()
  {
  }

  public void periodic()
  {
    m_elevator.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    m_elevator.simIterate();
  }

  /**
   * Reset the encoder to the lowest position when the current threshhold is reached. Should be used when the Elevator
   * position is unreliable, like startup. Threshhold is only detected if exceeded for 0.4 seconds, and the motor moves
   * less than 2 degrees per second.
   *
   * @param threshhold The current threshhold held when the Elevator is at it's hard limit.
   * @return
   */
  public Command homing(Current threshhold)
  {
      Debouncer       currentDebouncer  = new Debouncer(0.4); // Current threshold is only detected if exceeded for 0.4 seconds.
      Voltage runVolts          = Volts.of(-2); // Volts required to run the mechanism down. Could be positive if the mechanism is inverted.
      Distance limitHit          = Climber.hardLowerLimit;  // Limit which gets hit. Could be the lower limit if the volts makes the arm go down.
      AngularVelocity velocityThreshold = DegreesPerSecond.of(2); // The maximum amount of movement for the arm to be considered "hitting the hard limit".
      return Commands.startRun(motor::stopClosedLoopController, // Stop the closed loop controller
                      () -> motor.setVoltage(runVolts)) // Set the voltage of the motor
              .until(() -> currentDebouncer.calculate(motor.getStatorCurrent().gte(threshhold) &&
                      motor.getMechanismVelocity().abs(DegreesPerSecond) <=
                              velocityThreshold.in(DegreesPerSecond)))
              .finallyDo(() -> {
                  motor.setEncoderPosition(limitHit);
                  motor.startClosedLoopController();
              });
  }

  public Command elevCmd(double dutycycle)
  {
    return m_elevator.set(dutycycle);
  }

  public Command setHeight(Distance height)
  {
    return m_elevator.setHeight(height);
  }

  public Command sysId()
  {
    return m_elevator.sysId(Volts.of(12), Volts.of(12).per(Second), Second.of(30));
  }
}