// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.systems;

import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretFlywheelSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShooterTargetingSystem {

  private SwerveSubsystem m_swerve;
  private TurretSubsystem m_turret;
  private HoodSubsystem m_hood;
  private TurretFlywheelSubsystem m_flywheel;

  public ShooterTargetingSystem(
      SwerveSubsystem swerve,
      TurretSubsystem turret,
      HoodSubsystem hood,
      TurretFlywheelSubsystem flywheel) {

    m_swerve = swerve;
    m_turret = turret;
    m_hood = hood;
    m_flywheel = flywheel;
  }
}
