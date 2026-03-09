package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDConstants;
import frc.robot.Constants.TurretConstants;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXSWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.units.YUnits;

public class TurretFlywheelSubsystem extends SubsystemBase {

  private final TalonFX flywheelMotor = new TalonFX(CanIDConstants.turretFlywheelID);
  private final TalonFX flywheelFollowerMotor = new TalonFX(CanIDConstants.turretFlywheelFollowerID);
  private final SmartMotorControllerConfig motorConfig =
       new SmartMotorControllerConfig(this)
          .withClosedLoopController(0, 0, 0)// RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
          //don't change gears for SA comp //acutal gear ratio: 22:18 TT
          .withIdleMode(MotorMode.COAST)
          .withTelemetry("FlywheelMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(60))//For flywheel, stator current lm can be 60A/80A //talon fx can handle up to 260A
          .withMotorInverted(true)
          //.withClosedLoopRampRate(Seconds.of(0.25))
          //.withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new SimpleMotorFeedforward(0.08, 0.119, 0.015)) //thanks 3561!
          .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
          .withFollowers(Pair.of(flywheelFollowerMotor, true))
          .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController motor =
      new TalonFXWrapper(flywheelMotor, DCMotor.getKrakenX60(2), motorConfig);

  private final FlyWheelConfig flywheelConfig =
      new FlyWheelConfig(motor)
          .withDiameter(Inches.of(4))
          .withMass(Pounds.of(1))
          .withTelemetry("Flywheel", TelemetryVerbosity.HIGH);
          //.withSoftLimit(RPM.of(-5000), RPM.of(5000))
          //.withSpeedometerSimulation(RPM.of(7500));

  private final FlyWheel flywheel = new FlyWheel(flywheelConfig);

  public TurretFlywheelSubsystem() {}

  public AngularVelocity getRPM() {
    return flywheel.getSpeed();
  }

  
 

  public Command setRPM(double rpm) {
    return flywheel.setSpeed(RPM.of(rpm));
  }

  public Command setDutyCycle(double dutyCycle) {
    return flywheel.set(dutyCycle);
  }

  public void stop() {
    flywheel.set(0);
  }

  public void periodic()
  {
    flywheel.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    flywheel.simIterate();
  }
  /********************AlignToGoal*************************** */
  public void setRPM(LinearVelocity newHorizontalSpeed)
  {
    flywheel.setMeasurementVelocitySetpoint(newHorizontalSpeed);
  }

  /**********************************Shoot Cmd**************************** */
  // public boolean readyToShoot(AngularVelocity tolerance)
  // {
  //   if (motor.getMechanismSetpointVelocity().isEmpty())
  //   {return false;}
  //   return motor.getMechanismVelocity().isNear(motor.getMechanismSetpointVelocity().orElseThrow(), tolerance);
  // }

  // public void setTargetRPM(double rpm){
  //   motor.setVelocity(RPM.of(rpm));//Using SMC?
  // }
}
// public Command sysId() {
//     return flywheel.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));
//   }

// public Command setVelocity(LinearVelocity speed) {
  //   return flywheel.setSpeed(
  //       RPM.of(speed.in(YUnits.SandwichPerSecond) * TurretConstants.wheelDiameter));
  // }
