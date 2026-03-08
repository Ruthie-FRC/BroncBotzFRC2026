/*package frc.robot.systems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Setpoints.Indexer;
import frc.robot.Setpoints.Turret.Flywheel;
import frc.robot.commands.AlignToGoal;
import frc.robot.subsystems.AgitatorSubsystem;
//import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Turret.HoodSubsystem;
import frc.robot.subsystems.Turret.IndexerSubsystem;
import frc.robot.subsystems.Turret.KickerSubsystem;
import frc.robot.subsystems.Turret.TurretFlywheelSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;

import frc.robot.systems.ShooterTargetingSystem.Shot;
import yams.mechanisms.velocity.FlyWheel;

public class ScoringSystem {

  private IndexerSubsystem m_indexer;
  private IntakeArmSubsystem m_intakeArm;
  private IntakeRollerSubsystem m_intakeRollers;
  private SwerveSubsystem m_swerve;
  private TurretSubsystem m_turret;
  private HoodSubsystem m_hood;
  private AgitatorSubsystem m_agitator;
  private TurretFlywheelSubsystem m_flywheel;
  private KickerSubsystem m_kicker;
  private ShooterTargetingSystem m_shooterAimer; 
  private AlignToGoal aim;
  //private ShootOnTheMoveCommand SOTM;

  public ScoringSystem(
      IndexerSubsystem indexer,
      IntakeArmSubsystem intakeArm,
      IntakeRollerSubsystem intakeRoller,
      SwerveSubsystem swerve,
      TurretSubsystem turret, TurretFlywheelSubsystem turretFlywheel, HoodSubsystem hood, KickerSubsystem kicker, AgitatorSubsystem agitator) {
    m_indexer = indexer;
    m_intakeArm = intakeArm;
    m_intakeRollers = intakeRoller;
    m_swerve = swerve;
    m_turret = turret;
    m_hood = hood;
    m_kicker = kicker;
    m_flywheel = turretFlywheel;
    m_agitator = agitator;
    //SOTM = sotm;
  }
  

  public Command aim(){
    return null;
  }
  

  // public Command score() {
  //   // this one included turret tracking
  //   return SOTM;
  // }

  public Command shootBall() {

    // just transfer and shoot
    return m_turret.setAngle(Degrees.zero()).alongWith(m_flywheel.setVelocity(MetersPerSecond.of(8.44)), m_kicker.kickerShoot(), m_indexer.indexShoot());
    
  }
}
*/