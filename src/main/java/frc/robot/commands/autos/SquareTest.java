package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignAndShoot;
import frc.robot.commands.ConsumeNote;
import frc.robot.commands.GoTo;
import frc.robot.commands.GoTo;
import frc.robot.commands.GoTo;
import frc.robot.subsystems.IntakeIndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SquareTest extends SequentialCommandGroup {

  public SquareTest(SwerveSubsystem drivebase) {
    addCommands(
      new InstantCommand(() ->
        drivebase.resetOdometry(
          new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0.00))
        )
      ),
      new GoTo(
        new Pose2d(new Translation2d(1, 0), Rotation2d.fromDegrees(90)),
        drivebase
      ),
      new GoTo(
        new Pose2d(new Translation2d(1, 1), Rotation2d.fromDegrees(180)),
        drivebase
      ),
      new GoTo(
        new Pose2d(new Translation2d(0, 1), Rotation2d.fromDegrees(270)),
        drivebase
      ),
      new GoTo(
        new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
        drivebase
      )
    );
    addRequirements(drivebase);
  }
}
