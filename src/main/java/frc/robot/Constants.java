// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double maxSpeed = 4.5; //meters per second

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND = 0.05;
  }

  public static class CanIDConstants {

    public static final int climberCanID = 10;
    public static final int intakeRollerID = 20;
    public static final int turretID = 30;
    public static final int intakeArmID = 40;
    public static final int swerveID = 50;
    public static int turretFlywheelID = 31;

  

  }
  public static class Climber {

    public static final String           motorTelemetryName = "ExponentiallyProfiledElevatorMotor";
    public static final String           mechTelemetryName  = "ExponentiallyProfiledElevator";
    public static final SparkMax         elevatorMotor      = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);
    ///  Configuration Options
    public static final DCMotor          dcMotor            = DCMotor.getNEO(1);
    public static final Distance         chainPitch         = Inches.of(0.25);
    public static final int              toothCount         = 22;
    public final static Distance         circumference      = chainPitch.times(toothCount);
    public static final Distance         radius             = circumference.div(2 * Math.PI);
    public static final MechanismGearing gearing            = new MechanismGearing(GearBox.fromReductionStages(3, 4));
    public static final Mass             weight             = Pounds.of(16);
    /*
    * Using a measuring tape, where 0 cm marks the elevator at its lowest point,
    * you can measure the height to determine the starting position reference.
    */
    public static final Distance            startingHeight      = Meters.of(0);
    /*
    * To find these limits, measure the starting height relative to the elevator's lowest position using a measuring tape or ruler.
    */
    public static final Distance            softLowerLimit     = Meters.of(0);
    public static final Distance            softUpperLimit     = Meters.of(2);
    /*
    * These are the real "limits" of the robot shown in simulation.
    */
    public static final Distance            hardLowerLimit     = Meters.of(0);
    public static final Distance            hardUpperLimit     = Meters.of(3);
  }

  public static class Intake {

    public static final String           motorTelemetryName = "ExponentiallyProfiledArmMotor";
    public static final String           mechTelemetryName  = "ExponentiallyProfiledArm";
    public static final SparkMax         armMotor           = new SparkMax(1, MotorType.kBrushless);
    ///  Configuration Options
    public static final DCMotor          dcMotor            = DCMotor.getNEO(1);
    public static final MechanismGearing gearing            = new MechanismGearing(7);
    public static final Mass             weight             = Pounds.of(10);
    public static final Distance         length             = Feet.of(2);
    /*
    * Using the protractor, where 0deg on the protractor is when the arm is parallel to the ground,
    * you can measure where the starting angle should be.
    */
    public static final Angle            startingAngle      = Degrees.of(30);
    /*
    * To find these limits measure the starting angle relative to when the arm is parallel to the ground using a protractor.
    */
    public static final Angle            softLowerLimit     = Degrees.of(-20);
    public static final Angle            softUpperLimit     = Degrees.of(100);
    /*
    * These are the real "limits" of the robot shown in simulation.
    */
    public static final Angle            hardLowerLimit     = Degrees.of(-30);
    public static final Angle            hardUpperLimit     = Degrees.of(110);

  }



}
