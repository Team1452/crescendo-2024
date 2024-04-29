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

public class SpeakerTopRedShootPreloadAndFetch extends SequentialCommandGroup {

  public SpeakerTopRedShootPreloadAndFetch(
    IntakeIndexerSubsystem intakeIndexerSubsystem,
    ShooterSubsystem shooterSubsystem,
    SwerveSubsystem drivebase
  ) {
    addCommands(
      new InstantCommand(() ->
        drivebase.resetOdometry(
          new Pose2d(
            new Translation2d(15.88, 6.75),
            Rotation2d.fromDegrees(120)
          )
        )
      ),
      new AlignAndShoot(drivebase, shooterSubsystem, intakeIndexerSubsystem),
      new GoTo(
        new Pose2d(new Translation2d(14.95, 6.81), Rotation2d.fromDegrees(0)),
        drivebase
      ),
      new ConsumeNote(drivebase, intakeIndexerSubsystem),
      new GoTo(
        new Pose2d(new Translation2d(15.88, 6.75), Rotation2d.fromDegrees(120)),
        drivebase
      ),
      new AlignAndShoot(drivebase, shooterSubsystem, intakeIndexerSubsystem)
    );
  }
}
