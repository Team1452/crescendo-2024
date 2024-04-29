package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignAndShoot;
import frc.robot.commands.ConsumeNote;
import frc.robot.commands.GoTo;
import frc.robot.subsystems.IntakeIndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SpeakerTopBlueShootPreloadAndFetch extends SequentialCommandGroup {

  public SpeakerTopBlueShootPreloadAndFetch(
    IntakeIndexerSubsystem intakeIndexerSubsystem,
    ShooterSubsystem shooterSubsystem,
    SwerveSubsystem drivebase
  ) {
    addCommands(
      new InstantCommand(() ->
        drivebase.resetOdometry(
          new Pose2d(
            new Translation2d(0.95, 6.57),
            Rotation2d.fromDegrees(-120)
          )
        )
      ),
      new AlignAndShoot(drivebase, shooterSubsystem, intakeIndexerSubsystem),
      new GoTo(
        new Pose2d(new Translation2d(2.15, 6.83), Rotation2d.fromDegrees(0)),
        drivebase
      )
        .withTimeout(2.5),
      new ConsumeNote(drivebase, intakeIndexerSubsystem).withTimeout(5),
      new GoTo(
        new Pose2d(new Translation2d(0.95, 6.57), Rotation2d.fromDegrees(-120)),
        drivebase
      )
        .withTimeout(2.5),
      new AlignAndShoot(drivebase, shooterSubsystem, intakeIndexerSubsystem)
    );
  }
}
