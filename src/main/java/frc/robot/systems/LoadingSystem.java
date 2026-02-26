package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Setpoints;
import frc.robot.Constants.Agitator;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Turret.IndexerSubsystem;
import frc.robot.subsystems.Turret.KickerSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;

public class LoadingSystem {

  private IndexerSubsystem m_indexer;
  private IntakeArmSubsystem m_intakeArm;
  private IntakeRollerSubsystem m_intakeRollers;
  private SwerveSubsystem m_swerve;
  private TurretSubsystem m_turret;
  private AgitatorSubsystem m_agitator;
  private KickerSubsystem m_kicker;
  
  public LoadingSystem(   IndexerSubsystem indexer,
                          IntakeArmSubsystem intakeArm,
                          IntakeRollerSubsystem intakeRoller,
                          SwerveSubsystem swerve,
                          TurretSubsystem turret, 
                          AgitatorSubsystem agitator,
                          KickerSubsystem kicker) {
      m_indexer = indexer;
      m_intakeArm = intakeArm;
      m_intakeRollers = intakeRoller;
      m_swerve = swerve;
      m_turret = turret;
      m_agitator = agitator;
      m_kicker = kicker;
  }
  
  public Command intakeDown(){//move the intakeArm to intake angle
    return m_intakeArm.setAngle(Setpoints.Intake.intakeArmAngleIn)
                      .until(m_intakeArm.aroundAngle((Setpoints.Intake.intakeArmAngleIn)));
  }

  public Command intakeBalls() {//at intake angle, spin intake rollers, spin agitator
    return  intakeDown().andThen(m_intakeArm.setAngle(Setpoints.Intake.intakeArmAngleIn)
                        .alongWith(m_intakeRollers.in(), m_agitator.in()));//onchange
  }

  public Command stopIntakeBalls(){//stop
    return m_intakeArm.setAngle(Setpoints.Intake.intakeArmStartAngle)
                      .alongWith(m_intakeRollers.stop(), m_agitator.stop());//simulate? speed
  }

  public Command autointakeBalls() {
    // intake the balls
    return null;
  }

  public Command transferBalls() {// transfer balls to the shooter(indexer, kicker)
    return m_agitator.in().alongWith(m_indexer.indexShoot(), m_kicker.kickerShoot());
  }

  public Command intakeHumanPlayer() {
    // when we intake from human player
    return null;
  }

  //Hold cmd? bc set power to zero the arm would drop due to gravity
}


