package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;

public class Setpoints
{




  public static class Climber
  {

  }

  public static class Intake
  {

    public static Angle intakeArmStartAngle = Degrees.of(0);

  }

  public static class Indexer
  {

  }

  public static class Turret
  {

    public static class Hood {
      public static final Angle defaultHoodAngle = Degrees.of(30);
    }
    public static class Piviot {
      public static final Angle startTurretAngle = Degree.of(0);
    }

  }

  public static class AutoDriving
  {

      /* how to create offset
   *         public static final Transform2d offset = new Transform2d(Inches.of(24).in(Meters),
                                                                 Inches.of(0).in(Meters),
                                                                 Rotation2d.fromDegrees(0));
   */
  // x + front ->, y + left 

    //create Classes here


    }
  }



