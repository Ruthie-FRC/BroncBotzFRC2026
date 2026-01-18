package frc.robot.systems;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretFlywheelSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShooterTargetingSystem {

    private SwerveSubsystem m_swerve;
    private TurretSubsystem m_turret;
    private HoodSubsystem m_hood;
    private TurretFlywheelSubsystem m_flywheel;

    public ShooterTargetingSystem(SwerveSubsystem swerve, TurretSubsystem turret, HoodSubsystem hood, TurretFlywheelSubsystem flywheel){

        m_swerve = swerve;
        m_turret = turret;
        m_hood = hood;
        m_flywheel = flywheel;

    }
}
