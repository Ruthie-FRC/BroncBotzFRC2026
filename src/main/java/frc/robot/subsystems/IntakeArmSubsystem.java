package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
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
import yams.motorcontrollers.simulation.Sensor;

public class IntakeArmSubsystem extends SubsystemBase {


  // Vendor motor controller object
  private SparkMax m_motor = new SparkMax(Constants.CanIDConstants.intakeArmID, MotorType.kBrushless);
  private SparkMax m_followerMotor = new SparkMax(Constants.CanIDConstants.intakeArmFollowerID, MotorType.kBrushless);
  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Feedback Constants (PID Constants)
  .withClosedLoopController(GroundConstants.kP, GroundConstants.kI, GroundConstants.kD)
  .withSimClosedLoopController(GroundConstants.ksimP, GroundConstants.ksimI, GroundConstants.ksimD)
  // Feedforward Constants
  .withFeedforward(new ArmFeedforward(GroundConstants.kS, GroundConstants.kG, GroundConstants.kV))
  .withSimFeedforward(new ArmFeedforward(GroundConstants.ksimS, GroundConstants.ksimG, GroundConstants.ksimV))
  // Gearing from the motor rotor to final shaft.
  // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to the gearbox attached to your motor.
  .withTelemetry("IntakeArmMotor", TelemetryVerbosity.HIGH)
  .withGearing(GroundConstants.gearing)
  // Motor properties to prevent over currenting
  .withMotorInverted(false)
  .withIdleMode(MotorMode.BRAKE)
  .withStartingPosition(GroundConstants.startingPosition)
  .withStatorCurrentLimit(Amps.of(40))
  .withClosedLoopRampRate(Seconds.of(0.25))
  .withOpenLoopRampRate(Seconds.of(0.25))
  .withFollowers(Pair.of(m_followerMotor, true))
  .withExternalEncoder(m_motor.getAbsoluteEncoder())
  .withExternalEncoderInverted(false)
  .withUseExternalFeedbackEncoder(true)
  .withExternalEncoderGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
  .withExternalEncoderZeroOffset(Degrees.of(0));
  
  
  
  //-same thing as ArmConfig.withHorizontalZero()
  
  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController = new SparkWrapper(m_motor, DCMotor.getNEO(2), smcConfig);
  
 
  private ArmConfig armCfg = new ArmConfig(sparkSmartMotorController)
  // Soft limit is applied to the SmartMotorControllers PID
  .withSoftLimits(GroundConstants.softLowerLimit, GroundConstants.softUpperLimit)
  // Hard limit is applied to the simulation.
  .withHardLimit(GroundConstants.hardLowerLimit, GroundConstants.hardUpperLimit)
  .withStartingPosition(Degrees.of(0))
  // .withHorizontalZero(Degrees.of(15))

  // Length and mass of your arm for sim.
  .withLength(GroundConstants.length)
  .withMass(GroundConstants.weight)
  // Telemetry name and verbosity for the arm.
  .withTelemetry("IntakeArm", TelemetryVerbosity.HIGH);

  
  // Arm Mechanism
  private Arm arm = new Arm(armCfg);

  /** Creates a new ExampleSubsystem. */
  public IntakeArmSubsystem() {
   //sparkSmartMotorController.synchronizeRelativeEncoder(); 
  }
 

   /**
   * Set the angle of the arm.
   * @param angle Angle to go to.
   */
  
  public Command setAngle(Angle angle) {
    return arm.setAngle(angle);
    //.until(arm.isNear(angle, Degrees.of(OutakeConstants.kArmAllowableError)));
  }
  
  public Command setAngle(Supplier<Angle> angleSupplier) {
    return arm.setAngle(angleSupplier);
  }

  public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
    return arm.set(dutyCycleSupplier);
  }
  
  public Command setDutyCycle(double dutyCycle) {
    return arm.set(dutyCycle);
  }
  /**
   * Move the arm up and down.
   * @param dutycycle [-1, 1] speed to set the arm too.
   */
  public Command set(double dutycycle) { return arm.set(dutycycle);} //sparkMaxController.getDutyCycle();
  //DutyCycleEncoder m_encoderFR = new DutyCycleEncoder(0, 4.0, 2.0); 0-DIO channel 0

  /**
   * Run sysId on the {@link Arm}
   */
  public Command sysId() { 
    return arm.sysId(Volts.of(4.5), Volts.of(0.5).per(Second), Seconds.of(4));
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


public BooleanSupplier aroundAngle(Angle angle) {
  return arm.isNear(angle, GroundConstants.tolerationAngle);
}

}

/*
 * Things to be done
 * Tuning
 * LaserCan
 */