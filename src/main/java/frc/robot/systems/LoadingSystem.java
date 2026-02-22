package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Agitator;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Turret.IndexerSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;

public class LoadingSystem {

  private IndexerSubsystem m_indexer;
  private  IntakeArmSubsystem m_intakeArm;
  private IntakeRollerSubsystem m_intakeRollers;
  private SwerveSubsystem m_swerve;
  private TurretSubsystem m_turret;
  private AgitatorSubsystem m_agitator;
  
    public LoadingSystem( IndexerSubsystem indexer,
                          IntakeArmSubsystem intakeArm,
                          IntakeRollerSubsystem intakeRoller,
                          SwerveSubsystem swerve,
                          TurretSubsystem turret, AgitatorSubsystem agitator) {
      m_indexer = indexer;
      m_intakeArm = intakeArm;
      m_intakeRollers = intakeRoller;
      m_swerve = swerve;
      m_turret = turret;
      m_agitator = agitator;
    }
  
    private Command intakeDown(){
      return null;
  }

  private Command intakeBalls() {
    // intake the balls
    return intakeDown().alongWith(m_intakeRollers.in(), m_agitator.in());
  }

  private Command autointakeBalls() {
    // intake the balls
    return null;
  }

  private Command transferBalls() {
    // transfer balls to the shooter
    return null;
  }

  private Command intakeHumanPlayer() {
    // when we intake from human player
    return null;
  }
}
