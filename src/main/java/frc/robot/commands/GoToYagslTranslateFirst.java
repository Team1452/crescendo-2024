package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class GoToYagslTranslateFirst extends SequentialCommandGroup {

  public GoToYagslTranslateFirst(
    Pose2d targetPose,
    SwerveSubsystem driveSubsystem
  ) {
    addCommands(
      driveSubsystem.driveToPose(
        new Pose2d(
          targetPose.getTranslation(),
          driveSubsystem.getPose().getRotation()
        )
      ),
      driveSubsystem.driveToPose(targetPose)
    );
    addRequirements(driveSubsystem);
  }
}
