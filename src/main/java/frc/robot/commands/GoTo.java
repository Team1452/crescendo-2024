package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class GoTo extends Command {

  private final SwerveSubsystem driveSubsystem;
  private final Pose2d targetPose;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GoTo(Pose2d targetPose, SwerveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.targetPose = targetPose;
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

    var dTranslation = targetPose
      .getTranslation()
      .minus(currentPose.getTranslation());

    double vxMetersPerSecond = dTranslation.getX();
    double vyMetersPerSecond = dTranslation.getY();
    double omegaRadiansPerSecond = targetPose
      .getRotation()
      .minus(currentPose.getRotation())
      .getRadians();

    System.out.println(
      "Target angle: " +
      targetPose.getRotation().getDegrees() +
      ". Current angle: " +
      currentPose.getRotation().getDegrees() +
      ". Diff: " +
      Math.toDegrees(omegaRadiansPerSecond)
    );

    double speed = Math.hypot(vxMetersPerSecond, vyMetersPerSecond);

    // Clamp max speed
    if (speed > AutoConstants.kMaxSpeedMetersPerSecond) {
      vxMetersPerSecond *= AutoConstants.kMaxSpeedMetersPerSecond / speed;
      vyMetersPerSecond *= AutoConstants.kMaxSpeedMetersPerSecond / speed;
    }

    var translation = new Translation2d(vxMetersPerSecond, vyMetersPerSecond);
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
