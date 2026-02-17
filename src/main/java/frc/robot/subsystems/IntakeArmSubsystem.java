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
import frc.robot.Constants.IntakeConstants;
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
  // Telemetry name and verbosity level
  .withTelemetry("IntakeArm", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft.
  // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to the gearbox attached to your motor.
  .withGearing(new MechanismGearing(new GearBox(GroundConstants.gearbox), new Sprocket(GroundConstants.sprocket)))
  // Motor properties to prevent over currenting.
  .withMotorInverted(false)
  .withIdleMode(MotorMode.BRAKE)
  .withStartingPosition(GroundConstants.kStartingPose)
  .withStatorCurrentLimit(GroundConstants.statorCurrentLimit)
  .withClosedLoopRampRate(Seconds.of(0.25))
  .withOpenLoopRampRate(Seconds.of(0.25))
  ;
  
  //.withZeroOffset(Degrees.of(0));-same thing as ArmConfig.withHorizontalZero()
  
  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController = new SparkWrapper(m_motor, DCMotor.getNEO(1), smcConfig);
 
  private ArmConfig armCfg = new ArmConfig(sparkSmartMotorController)
  // Soft limit is applied to the SmartMotorControllers PID
  .withSoftLimits(GroundConstants.softLimitMin, GroundConstants.softLimitMax)
  // Hard limit is applied to the simulation.
  .withHardLimit(GroundConstants.hardLimitMin, GroundConstants.hardLimitMax)
  .withStartingPosition(Degrees.of(90))
  // Length and mass of your arm for sim.
  .withLength(GroundConstants.armLength)
  .withMass(GroundConstants.armMass)
  // Telemetry name and verbosity for the arm.
  .withTelemetry("IntakeArm", TelemetryVerbosity.HIGH);

  
  // Arm Mechanism
  private Arm arm = new Arm(armCfg);

  /** Creates a new ExampleSubsystem. */
  public IntakeArmSubsystem() {
   //sparkSmartMotorController.synchronizeRelativeEncoder(); 
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

  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    arm.simIterate();
  }

  public Angle getAngle() {
    return arm.getAngle();
  }

}

/*
 * Things to be done
 * Tuning
 * LaserCan
 */