// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import yams.units.YUnits;

public class Setpoints {

  public static class Climber {}

  public static class Intake {

    public static Angle intakeArmStartAngle = Degrees.of(120);
    public static Angle intakeAngle = Degrees.of(0);
  }

  public static class Indexer {
    // indexSubsystem and agitator
    public static final AngularVelocity startFlywheelAngle = YUnits.RPY.of(0);
  }

  public static class Turret {

    public static class Hood {
      public static final Angle startHoodAngle = Degrees.of(30);
      public static final Angle lowerHoodAngle = Degrees.of(0);
      public static final Angle higherHoodAngle = Degrees.of(60);
    }

    public static class Pivot {
      // these are random, check before running on real robot to avoid breaking
      public static final Angle startTurretAngle = Degree.of(180);
      public static final Angle leftTurretAngle = Degree.of(90);
      public static final Angle rightTurretAngle = Degree.of(270);
    }

    public static class Flywheel {
      public static final AngularVelocity startFlywheelAngle = YUnits.RPY.of(0);
    }
  }

  public static class AutoDriving {

    /* how to create offset
    *         public static final Transform2d offset = new Transform2d(Inches.of(24).in(Meters),
                                                                  Inches.of(0).in(Meters),
                                                                  Rotation2d.fromDegrees(0));
    */
    // x + front ->, y + left

    // create Classes here

  }
}
