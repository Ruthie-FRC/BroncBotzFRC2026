package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ScoringSystem {

  private IndexerSubsystem m_indexer;
  private IntakeArmSubsystem m_intakeArm;
  private IntakeRollerSubsystem m_intakeRollers;
  private SwerveSubsystem m_swerve;
  private TurretSubsystem m_turret;

  public ScoringSystem(
      IndexerSubsystem indexer,
      IntakeArmSubsystem intakeArm,
      IntakeRollerSubsystem intakeRoller,
      SwerveSubsystem swerve,
      TurretSubsystem turret) {
    m_indexer = indexer;
    m_intakeArm = intakeArm;
    m_intakeRollers = intakeRoller;
    m_swerve = swerve;
    m_turret = turret;
  }

  private static Command score() {
    // this one included turret tracking
    return null;
  }

  private static Command shootBall() {
    // just transfer and shoot
    return null;
  }
}
