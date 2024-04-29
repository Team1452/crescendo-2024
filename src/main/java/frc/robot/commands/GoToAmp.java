package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class GoToAmp extends Command {

  private final SwerveSubsystem driveSubsystem;

  private Pose2d targetPose;

  private final double targetFiducialId = DriverStation
      .getAlliance()
      .orElseGet(() -> Alliance.Blue) ==
    Alliance.Red
    ? 5.0 // Centered tag for red.
    : 6.0; // Centered tag for blue.

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GoToAmp(SwerveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = driveSubsystem.getPose();

    var llresults = LimelightHelpers.getLatestResults("limelight-shooter");

    for (LimelightTarget_Fiducial target : llresults.targetingResults.targets_Fiducials) {
      if (target.fiducialID == targetFiducialId) {
        Pose2d targetPoseRobotSpace = target.getTargetPose_RobotSpace2D();
        targetPose = targetPoseRobotSpace.relativeTo(driveSubsystem.getPose());

        System.out.println("targetPoseRobotSpace: " + targetPoseRobotSpace);
        System.out.println("driveSubsystem Pose: " + driveSubsystem.getPose());
        // targetPose =
        //   new Pose2d(
        //     new Translation2d(
        //       targetPoseRobotSpace.getX() + driveSubsystem.getPose().getX(),
        //       targetPoseRobotSpace.getY() + driveSubsystem.getPose().getY()
        //     ),
        //     new Rotation2d(
        //       targetPoseRobotSpace.getRotation().getDegrees() +
        //       driveSubsystem.getPose().getRotation().getDegrees()
        //     )
        //   );
      }
    }

    Twist2d twist = currentPose.log(targetPose);

    double vxMetersPerSecond = twist.dx;
    double vyMetersPerSecond = twist.dy;
    double omegaRadiansPerSecond = twist.dtheta;

    double speed = Math.hypot(vxMetersPerSecond, vyMetersPerSecond);

    // Clamp max speed
    if (speed > AutoConstants.kMaxSpeedMetersPerSecond) {
      vxMetersPerSecond *= AutoConstants.kMaxSpeedMetersPerSecond / speed;
      vyMetersPerSecond *= AutoConstants.kMaxSpeedMetersPerSecond / speed;
    }

    double angleRadians = Math.abs(speed) > 1e-6
      ? Math.atan2(vyMetersPerSecond, vxMetersPerSecond)
      : 0;

    var translation = new Translation2d(
      speed,
      Rotation2d.fromRadians(angleRadians)
    );
    driveSubsystem.drive(translation, omegaRadiansPerSecond, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d currentPose = driveSubsystem.getPose();
    Transform2d diff = targetPose.minus(currentPose);
    return (
      Math.abs(diff.getX()) < 0.1 &&
      Math.abs(diff.getY()) < 0.1 &&
      Math.abs(diff.getRotation().getDegrees()) < 10
    );
  }
}
