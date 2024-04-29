package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class GoToYagsl extends SequentialCommandGroup {

  public GoToYagsl(Pose2d targetPose, SwerveSubsystem driveSubsystem) {
    addCommands(driveSubsystem.driveToPose(targetPose));
    addRequirements(driveSubsystem);
  }
}
