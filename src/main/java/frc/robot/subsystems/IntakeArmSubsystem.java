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

    private SparkMax m_motor = new SparkMax(Constants.CanIDConstants.intakeArmID, MotorType.kBrushless);
    private SparkMax m_followerMotor = new SparkMax(Constants.CanIDConstants.intakeArmFollowerID, MotorType.kBrushless);
    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(1.35, 0, 0)
            .withSimClosedLoopController(10, 0, 0)
            .withFeedforward(new ArmFeedforward(0.15, 0, 0))
            .withSimFeedforward(new ArmFeedforward(0.25, 0, 0.25))
            .withTelemetry("IntakeArmMotor", TelemetryVerbosity.HIGH)
            .withGearing(GroundConstants.gearing)
            .withMotorInverted(true)
            .withIdleMode(MotorMode.BRAKE)
            .withStartingPosition(Setpoints.Intake.intakeArmStartAngle)
            .withStatorCurrentLimit(Amps.of(40))
            .withFollowers(Pair.of(m_followerMotor, true));
            // .withExternalEncoder(m_motor.getAbsoluteEncoder())
            // .withExternalEncoderInverted(true)
            // .withUseExternalFeedbackEncoder(true)
            // .withExternalEncoderZeroOffset(Degrees.of(87.089));


    //-same thing as ArmConfig.withHorizontalZero()

    // Create our SmartMotorController from our Spark and config with the NEO.
    private SmartMotorController sparkSmartMotorController = new SparkWrapper(m_motor, DCMotor.getNEO(2), smcConfig);


    private ArmConfig armCfg = new ArmConfig(sparkSmartMotorController)
            // Soft limit is applied to the SmartMotorControllers PID
            .withSoftLimits(GroundConstants.softLowerLimit, GroundConstants.softUpperLimit)
            // Hard limit is applied to the simulation.
            .withHardLimit(GroundConstants.hardLowerLimit, GroundConstants.hardUpperLimit)
            // .withHorizontalZero(Degrees.of(15))

            // Length and mass of your arm for sim.
            .withLength(GroundConstants.length)
            .withMass(GroundConstants.weight)
            // Telemetry name and verbosity for the arm.
            .withTelemetry("IntakeArm", TelemetryVerbosity.HIGH);


    // Arm Mechanism
    private Arm arm = new Arm(armCfg);

    /**
     * Creates a new ExampleSubsystem.
     */
    public IntakeArmSubsystem() {
        //sparkSmartMotorController.synchronizeRelativeEncoder();
    }

    public Angle getAngle() {
        return arm.getAngle();
    }

    public Command setAngleCommand(Angle angle) {
        return arm.setAngle(angle);
        //.until(arm.isNear(angle, Degrees.of(OutakeConstants.kArmAllowableError)));
    }

    public void setAngleSetpoint(Angle angle){
        arm.setMechanismPositionSetpoint(angle);
    }

    public Command setDutyCycleCommand(double dutyCycle) {
        return arm.set(dutyCycle);
    }

    public void setDutyCycleSetpoint(double dutyCycle){
        arm.setDutyCycleSetpoint(dutyCycle);
    }

    @Override
    public void periodic() {
        arm.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        arm.simIterate();
    }

    public BooleanSupplier aroundAngle(Angle angle) {
        return arm.isNear(angle, GroundConstants.tolerationAngle);
    }

    public Command setVoltageCommand(Voltage volt){
        return arm.setVoltage(volt);
    }
}
