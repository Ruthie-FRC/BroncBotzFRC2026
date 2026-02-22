package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Set;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Setpoints;
import frc.robot.Constants.GroundConstants;
import frc.robot.Setpoints.Intake;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.gearing.Sprocket;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

// TODO: Example with absolute encoders

/**
 * Exponentially profiled arm subsystem. The arm represented by this class does NOT have an absolute
 * encoder! This subsystem has a "self-homing" command, more details in the function description.
 */
public class IntakeArmSubsystem extends SubsystemBase {

  public static final SparkMax armMotor =
      new SparkMax(Constants.CanIDConstants.intakeArmID, MotorType.kBrushless);
  ///  Configuration Options
  public static final DCMotor dcMotor = DCMotor.getNEO(1);
  /*
   * This is the STARTING PID Controller for the Arm. If you are using a TalonFX or TalonFXS this will run on the motor controller itself.
   */
  private final ExponentialProfilePIDController pidController =
      new ExponentialProfilePIDController(
          Constants.IntakeConstants.kP,
          Constants.IntakeConstants.kI,
          Constants.IntakeConstants.kD,
          ExponentialProfilePIDController.createArmConstraints(
              Volts.of(12),
              dcMotor,
              IntakeConstants.weight,
              IntakeConstants.length,
              IntakeConstants.gearing));
  /*
   * This is the STARTING Feedforward for the Arm. If you are using a TalonFX or TalonFXS this will run on the motor controller itself.
   */
  private final ArmFeedforward armFeedforward =
      new ArmFeedforward(
          IntakeConstants.kS, IntakeConstants.kG, IntakeConstants.kV, IntakeConstants.kA);

  /** {@link SmartMotorControllerConfig} for the arm motor. */
  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withMotorInverted(false)
          .withIdleMode(MotorMode.BRAKE)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withGearing(IntakeConstants.gearing)
          .withStatorCurrentLimit(
              Amps.of(
                  40)) // Prevents our motor from continuously over-taxing itself when it is stuck.
          .withClosedLoopRampRate(
              Seconds.of(
                  0.25)) // Prevents our motor from rapid demand changes that could cause dramatic
          // voltage drops, and current draw.
          .withOpenLoopRampRate(Seconds.of(0.25)) // Same as above
          // SmartMotorControllerTelemetryConfig
          .withClosedLoopController(pidController)
          .withSimClosedLoopController(new ExponentialProfilePIDController(IntakeConstants.kPSim, 
                                                                            IntakeConstants.kISim, 
                                                                            IntakeConstants.kDSim, 
                                                                            ExponentialProfilePIDController.createArmConstraints(
                                                                                Volts.of(12),
                                                                                dcMotor,
                                                                                IntakeConstants.weight,
                                                                                IntakeConstants.length,
                                                                                IntakeConstants.gearing)))
          .withFeedforward(armFeedforward)
          .withSimFeedforward(new ArmFeedforward(IntakeConstants.kSSim, IntakeConstants.kGSim, IntakeConstants.kVSim, IntakeConstants.kASim) )
          .withSoftLimit(IntakeConstants.softLowerLimit, IntakeConstants.softUpperLimit)
          .withExternalEncoder(armMotor.getAbsoluteEncoder())
          .withExternalEncoderInverted(false)
          .withUseExternalFeedbackEncoder(true)
          .withExternalEncoderGearing(new MechanismGearing(GearBox.fromReductionStages(3,4)))
          .withExternalEncoderZeroOffset(Degrees.of(0));

  // encoder here

  /// Generic Smart Motor Controller with our options and vendor motor.
  private final SmartMotorController motor = new SparkWrapper(armMotor, dcMotor, motorConfig);
  /// Arm-specific options
  private ArmConfig m_config =
      new ArmConfig(motor)
          /*
           * Basic configuration options for the arm.
           */
          .withLength(IntakeConstants.length)
          .withMass(IntakeConstants.weight)
          .withStartingPosition(
              IntakeConstants
                  .startingAngle) // The starting position should ONLY be defined if you are NOT
          // using an absolute encoder.
          // .withHorizontalZero(Degrees.of(0)) // The horizontal zero should ONLY be defined if you
          // ARE using an absolute encoder.
          .withTelemetry("IntakeArm", TelemetryVerbosity.HIGH)
          /*
           * Simulation configuration options for the arm.
           */
          .withHardLimit(IntakeConstants.hardLowerLimit, IntakeConstants.hardUpperLimit);
  // Arm mechanism
  private final Arm arm = new Arm(m_config);

  public IntakeArmSubsystem() {}

  public void periodic() {
    arm.updateTelemetry();
  }

  public void simulationPeriodic() {
    arm.simIterate();
  }

  /**
   * Reset the encoder to the lowest position when the current threshhold is reached. Should be used
   * when the Arm position is unreliable, like startup. Threshhold is only detected if exceeded for
   * 0.4 seconds, and the motor moves less than 2 degrees per second.
   *
   * @param threshhold The current threshhold held when the Arm is at it's hard limit.
   * @return
   */
  public Command homing(Current threshhold) {
    Debouncer currentDebouncer =
        new Debouncer(0.4); // Current threshold is only detected if exceeded for 0.4 seconds.
    Voltage runVolts =
        Volts.of(
            2); // Volts required to run the mechanism up. Could be negative if the mechanism is
    // inverted.
    Angle limitHit =
        IntakeConstants
            .hardUpperLimit; // Limit which gets hit. Could be the lower limit if the volts makes
    // the arm go down.
    AngularVelocity velocityThreshold =
        DegreesPerSecond.of(
            2); // The maximum amount of movement for the arm to be considered "hitting the hard
    // limit".
    return Commands.startRun(
            motor::stopClosedLoopController, // Stop the closed loop controller
            () -> motor.setVoltage(runVolts)) // Set the voltage of the motor
        .until(
            () ->
                currentDebouncer.calculate(
                    motor.getStatorCurrent().gte(threshhold)
                        && motor.getMechanismVelocity().abs(DegreesPerSecond)
                            <= velocityThreshold.in(DegreesPerSecond)))
        .finallyDo(
            () -> {
              motor.startClosedLoopController();
            });
  }

  public Command armCmd(double dutycycle) {
    return arm.set(dutycycle);
  }

  public Command setAngle(Angle angle) {
    return arm.setAngle(angle);
  }

  public Command intakeAngle() {
    return arm.setAngle(Setpoints.Intake.intakeArmAngleOut);
  }

  public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
    return arm.set(dutyCycleSupplier);
  }

  public Command setDutyCycle(double dutyCycle) {
    return arm.set(dutyCycle);
  }

  public Command sysId() {
    return arm.sysId(Volts.of(3), Volts.of(3).per(Second), Second.of(30));
  }

  public Trigger atIntakeAngle() {
    return arm.isNear(Intake.intakeArmAngleIn, Degrees.of(IntakeConstants.tolerance));
  }
}
Copyright (c) FIRST and other WPILib contributors.
Open Source Software; you can modify and/or share it under the terms of
the WPILib BSD license file in the root directory of this project.


import yams.motorcontrollers.simulation.Sensor;

public class IntakeArmSubsystem extends SubsystemBase {


  // Vendor motor controller object
  private SparkMax m_motor = new SparkMax(Constants.CanIDConstants.intakeArmID, MotorType.kBrushless);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Feedback Constants (PID Constants)
  .withClosedLoopController(GroundConstants.kP, GroundConstants.kI, GroundConstants.kD, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
  .withSimClosedLoopController(GroundConstants.ksimP, GroundConstants.ksimI, GroundConstants.ksimD, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
  // Feedforward Constants
  .withFeedforward(new ArmFeedforward(GroundConstants.kS, GroundConstants.kG, GroundConstants.kV))
  .withSimFeedforward(new ArmFeedforward(GroundConstants.ksimS, GroundConstants.ksimG, GroundConstants.ksimV))
  // Gearing from the motor rotor to final shaft.
  // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to the gearbox attached to your motor.
  .withGearing(GroundConstants.gearing)
  // Motor properties to prevent over currenting.
  .withMotorInverted(false)
  .withIdleMode(MotorMode.BRAKE)
  .withStartingPosition(GroundConstants.startingPosition)
  .withStatorCurrentLimit(Amps.of(40))
  .withClosedLoopRampRate(Seconds.of(0.25))
  .withOpenLoopRampRate(Seconds.of(0.25))
  ;
  
  //.withZeroOffset(Degrees.of(0));-same thing as ArmConfig.withHorizontalZero()
  
  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController = new SparkWrapper(m_motor, DCMotor.getNEO(1), smcConfig);
 
  private ArmConfig armCfg = new ArmConfig(sparkSmartMotorController)
  // Soft limit is applied to the SmartMotorControllers PID
  .withSoftLimits(GroundConstants.softLowerLimit, GroundConstants.softUpperLimit)
  // Hard limit is applied to the simulation.
  .withHardLimit(GroundConstants.hardLowerLimit, GroundConstants.hardUpperLimit)

  // Length and mass of your arm for sim.
  .withLength(GroundConstants.length)
  .withMass(GroundConstants.weight)
  // Telemetry name and verbosity for the arm.
  .withTelemetry("IntakeArm", TelemetryVerbosity.HIGH);

  
  // Arm Mechanism
  private Arm m_Arm = new Arm(armCfg);

  /** Creates a new ExampleSubsystem. */
  public IntakeArmSubsystem() {
   //sparkSmartMotorController.synchronizeRelativeEncoder(); 
  }
 

   /**
   * Set the angle of the arm.
   * @param angle Angle to go to.
   */
  public Command setAngle(double angle) {
    return m_Arm.setAngle(Degrees.of(angle));
    //.until(arm.isNear(angle, Degrees.of(OutakeConstants.kArmAllowableError)));
}

  /**
   * Move the arm up and down.
   * @param dutycycle [-1, 1] speed to set the arm too.
   */
  public Command set(double dutycycle) { return m_Arm.set(dutycycle);} //sparkMaxController.getDutyCycle();
  //DutyCycleEncoder m_encoderFR = new DutyCycleEncoder(0, 4.0, 2.0); 0-DIO channel 0

  /**
   * Run sysId on the {@link Arm}
   */
  public Command sysId() { 
    return m_Arm.sysId(Volts.of(4.5), Volts.of(0.5).per(Second), Seconds.of(4));
  }


  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_Arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_Arm.simIterate();
  }

  public Angle getAngle() {
    return m_Arm.getAngle();
  }

}

/*
 * Things to be done
 * Tuning
 * LaserCan
 */