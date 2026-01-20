package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.CanIDConstants;

public class HardwareMap {
    
  public static final SparkMax agitator = new SparkMax(CanIDConstants.agitatorID, MotorType.kBrushless);
}
