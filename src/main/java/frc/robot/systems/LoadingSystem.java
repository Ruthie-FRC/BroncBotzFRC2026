package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class LoadingSystem {

  private IndexerSubsystem m_indexer;
  private IntakeArmSubsystem m_intakeArm;
  private IntakeRollerSubsystem m_intakeRollers;
  private SwerveSubsystem m_swerve;
  private TurretSubsystem m_turret;

  public LoadingSystem(
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

  private static Command intakeBalls() {
    // intake the balls
    return null;
  }

  private static Command transferBalls() {
    // transfer balls to the shooter
    return null;
  }

  private static Command intakeHumanPlayer() {
    // when we intake from human player
    return null;
  }
}
