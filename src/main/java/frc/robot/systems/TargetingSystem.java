package frc.robot.systems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

// import frc.robot.Setpoints.AutoScoring;
// import frc.robot.systems.field.FieldConstants.Reef;
// import frc.robot.systems.field.FieldConstants.ReefHeight;
import java.util.List;
import java.util.Map;

// targetting system should be able to select either left or right side of the branch
// then select what level we want
// that go to the nearest side of the reef and load.

public class TargetingSystem {

  private Trench targetBranch;
  // private ReefBranchLevel targetBranchLevel;
  private TrenchSide targetReefBranchSide = TrenchSide.CLOSEST;

  private List<Pose2d> trenchSetpoints = null;
  private List<Pose2d> allianceRelativeTrenchSetpoints = null;
  private Map<Pose2d, Trench> trenchPoseToBranchMap = null;

  private void initializeBranchPoses() {

    // trenchSetpoints = new ArrayList<>();
    // trenchPoseToBranchMap = new HashMap<>();
    // for (int branchPositionIndex = 0; branchPositionIndex < targetBranch.TrenchPose.size();
    // branchPositionIndex++)
    // {
    //   Map<ReefHeight, Pose3d> branchPosition = Reef.branchPositions.get(branchPositionIndex);
    //   Pose2d                  targetPose     = branchPosition.get(ReefHeight.L4).toPose2d();
    //   reefBranches.add(targetPose);
    //   reefPoseToBranchMap.put(targetPose, ReefBranch.values()[branchPositionIndex]);
    //   reefPoseToBranchMap.put(AllianceFlipUtil.flip(targetPose),
    // ReefBranch.values()[branchPositionIndex]);
    // }
    // allianceRelativeReefBranches =
    // reefBranches.stream().map(AllianceFlipUtil::apply).collect(Collectors.toList());
  }

  // private static int rightBranchOrdinal(ReefBranch branch)
  // {
  //   boolean isRight = (branch.ordinal()+1) % 2 == 0;
  //   return MathUtil.clamp(branch.ordinal() + (isRight ? 0 : 1),0,11);
  // }

  // private static int leftBranchOrdinal(ReefBranch branch)
  // {
  //   boolean isRight = (branch.ordinal()+1) % 2 == 0;
  //   return MathUtil.clamp(branch.ordinal() - (isRight ? 1 : 0),0,11);
  // }

  // public int getTargetBranchOrdinal()
  // {
  //   if (targetBranch != null)
  //   {
  //     switch (targetReefBranchSide)
  //     {
  //       case CLOSEST ->
  //       {
  //         return targetBranch.ordinal();
  //       }
  //       case RIGHT ->
  //       {
  //         return rightBranchOrdinal(targetBranch);
  //       }
  //       case LEFT ->
  //       {
  //         return leftBranchOrdinal(targetBranch);
  //       }
  //     }
  //   }
  //   return 0;
  // }

  // public TargetingSystem()
  // {
  //   RobotModeTriggers.autonomous().onFalse(Commands.runOnce(this::initializeBranchPoses));
  // }

  // public void setTarget(ReefBranch targetBranch, ReefBranchLevel targetBranchLevel)
  // {
  //   this.targetBranch = targetBranch;
  //   this.targetBranchLevel = targetBranchLevel;
  // }

  // public Command setTargetCommand(ReefBranch targetBranch, ReefBranchLevel targetBranchLevel)
  // {
  //   return Commands.runOnce(() -> setTarget(targetBranch, targetBranchLevel));
  // }

  // public Command setBranchCommand(ReefBranch branch)
  // {
  //   return Commands.runOnce(() -> {
  //     targetBranch = branch;
  //   });
  // }

  // public Command setBranchSide(ReefBranchSide side)
  // {
  //   return Commands.runOnce(() -> {
  //     targetReefBranchSide = side;
  //   });
  // }

  // public Command setBranchLevel(ReefBranchLevel level)
  // {
  //   return Commands.runOnce(() -> {
  //     targetBranchLevel = level;
  //   });
  // }

  // public ReefBranchLevel getTargetBranchLevel()
  // {
  //   return targetBranchLevel;
  // }

  // public ReefBranch getTargetBranch()
  // {
  //   return targetBranch;
  // }

  // public Command driveToCoralTarget(SwerveSubsystem swerveDrive)
  // {
  //   return (Commands.runOnce(() -> {
  //     swerveDrive.getSwerveDrive().field.getObject("target").setPose(getCoralTargetPose());
  //   })).andThen(swerveDrive.driveToPose(this::getCoralTargetPose));
  // }

  // public Command driveToAlgaeTarget(SwerveSubsystem swerveDrive)
  // {
  //   return (Commands.runOnce(() -> {
  //     swerveDrive.getSwerveDrive().field.getObject("target").setPose(getAlgaeTargetPose());
  //   })).andThen(swerveDrive.driveToPose(this::getAlgaeTargetPose));
  // }

  // public Pose2d getCoralTargetPose()
  // {
  //   Pose2d scoringPose = Pose2d.kZero;
  //   if (targetBranch != null)
  //   {
  //     Pose2d startingPose =
  // AllianceFlipUtil.apply(Reef.branchPositions.get(getTargetBranchOrdinal()).get(ReefHeight.L2)
  //                                                                      .toPose2d());
  //     SmartDashboard.putString("Targetted Coral Pose without Offset (Meters)",
  // startingPose.toString());
  //     scoringPose = startingPose.plus(AutoScoring.Reef.coralOffset);
  //     SmartDashboard.putString("Targetted Coral Pose with Offset (Meters)",
  // scoringPose.toString());

  //   }
  //   return scoringPose;
  // }

  // public Pose2d getAlgaeTargetPose()
  // {
  //   Pose2d scoringPose = Pose2d.kZero;
  //   if (targetBranch != null)
  //   {
  //     Pose2d startingPose =
  // AllianceFlipUtil.apply(Reef.branchPositions.get(rightBranchOrdinal(targetBranch))
  //
  // .get(ReefHeight.L2).toPose2d());
  //     SmartDashboard.putString("Targetted Algae Pose without Offset (Meters)",
  // startingPose.toString());
  //     scoringPose = startingPose.plus(AutoScoring.Reef.algaeOffset);
  //     SmartDashboard.putString("Targetted Algae Pose with Offset (Meters)",
  // scoringPose.toString());

  //   }
  //   return scoringPose;
  // }

  // public Pose2d autoTarget(Supplier<Pose2d> currentPose)
  // {
  //   if (reefBranches == null)
  //   {
  //     initializeBranchPoses();
  //   }

  //   Pose2d selectedTargetPose = currentPose.get().nearest(allianceRelativeReefBranches);
  //   targetBranch = reefPoseToBranchMap.get(selectedTargetPose);
  //   return selectedTargetPose;
  // }

  // public Command autoTargetCommand(Supplier<Pose2d> currentPose)
  // {
  //   return Commands.runOnce(() -> autoTarget(currentPose));
  // }

  // public void increaseBranch()
  // {
  //   if(targetBranch != null)
  //   {
  //     targetBranch = ReefBranch.values()[(targetBranch.ordinal() + 1) % 12];
  //     // System.out.println("Branch Selected: "+targetBranch.toString());
  //   }
  // }

  // public void printTargetPose(ReefBranchLevel level, ReefBranchSide side)
  // {

  //   targetBranchLevel = level;
  //   targetReefBranchSide = side;

  //  //autoTarget(pose);
  //  if(targetBranch!=null)
  //    System.out.println("Coral Branch: "+targetBranch.toString()+" Target Pose: " +
  // getCoralTargetPose().toString());
  // }

  // public void printTargetPose(ReefBranch branch, ReefBranchLevel level, ReefBranchSide side)
  // {
  //   targetBranch = branch;
  //   printTargetPose(level, side);
  // }

  // public void setCoralTargetOnField(SwerveSubsystem swerveDrive)
  // {
  //   swerveDrive.getSwerveDrive().field.getObject("target").setPose(getCoralTargetPose());
  // }

  // public void setAlgaeTargetOnField(SwerveSubsystem swerveDrive)
  // {
  //   swerveDrive.getSwerveDrive().field.getObject("target").setPose(getAlgaeTargetPose());
  // }

  public enum Trench {
    LEFT_ENTERENCE,
    LEFT_EXIT,
    RIGHT_ENTERENCE,
    RIGHT_EXIT
  }

  public enum TrenchSide {
    CLOSEST,
    RIGHT,
    LEFT
  }
}
