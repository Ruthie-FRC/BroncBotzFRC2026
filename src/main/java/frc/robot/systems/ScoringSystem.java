package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Setpoints.Indexer;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Turret.HoodSubsystem;
import frc.robot.subsystems.Turret.IndexerSubsystem;
import frc.robot.subsystems.Turret.KickerSubsystem;
import frc.robot.subsystems.Turret.TurretFlywheelSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.systems.ShooterTargetingSystem.Shot;

public class ScoringSystem {

  private IndexerSubsystem m_indexer;
  private IntakeArmSubsystem m_intakeArm;
  private IntakeRollerSubsystem m_intakeRollers;
  private SwerveSubsystem m_swerve;
  private TurretSubsystem m_turret;
  private HoodSubsystem m_hood;
  private TurretFlywheelSubsystem m_flywheel;
  private KickerSubsystem m_kicker;
  private ShooterTargetingSystem m_shooterAimer;

  public ScoringSystem(
      IndexerSubsystem indexer,
      IntakeArmSubsystem intakeArm,
      IntakeRollerSubsystem intakeRoller,
      SwerveSubsystem swerve,
      TurretSubsystem turret, HoodSubsystem hood, KickerSubsystem kicker) {
    m_indexer = indexer;
    m_intakeArm = intakeArm;
    m_intakeRollers = intakeRoller;
    m_swerve = swerve;
    m_turret = turret;
    m_hood = hood;
    m_kicker = kicker;
  }
  


  

  private Command score() {
    // this one included turret tracking
    Shot shot = ShooterTargetingSystem.getShotData(m_swerve.getPose(), m_swerve.getFieldVelocity(), 0); 
    return
    Commands.parallel(m_turret.setAngle(shot.getAngle()), m_hood.setAngle(shot.getAngle()), m_flywheel.setVelocity(shot.getVelocity()), m_indexer.indexIn(), m_kicker.kickerShoot());
  }

  private Command shootBall() {
    // just transfer and shoot
    return null;
  }
}
