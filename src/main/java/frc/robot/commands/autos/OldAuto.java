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

public class OldAuto extends SequentialCommandGroup {

  public OldAuto(
    IntakeIndexerSubsystem intakeIndexerSubsystem,
    ShooterSubsystem shooterSubsystem,
    SwerveSubsystem drivebase
  ) {
    addCommands(
      new InstantCommand(() -> System.out.println("REset odometry")),
      new InstantCommand(() ->
        drivebase.resetOdometry(
          new Pose2d(
            new Translation2d(1.32, 5.46),
            Rotation2d.fromDegrees(0.00)
          )
        )
      ),
      new InstantCommand(() -> System.out.println("Align and shoot")),
      new AlignAndShoot(drivebase, shooterSubsystem, intakeIndexerSubsystem),
      // new InstantCommand(() -> System.out.println("Move back")),
      // new GoTo(
      //   new Pose2d(new Translation2d(2.23, 5.46), Rotation2d.fromDegrees(0.00)),
      //   drivebase
      // ),
      new InstantCommand(() -> System.out.println("Consume note")),
      new ConsumeNote(drivebase, intakeIndexerSubsystem),
      new InstantCommand(() -> System.out.println("Go back to spekaer")),
      new GoTo(
        new Pose2d(new Translation2d(1.34, 5.46), Rotation2d.fromDegrees(0.00)),
        drivebase
      )
        .withTimeout(2.5),
      new InstantCommand(() ->
        System.out.println("Align and shoot w/ second note")
      ),
      new AlignAndShoot(drivebase, shooterSubsystem, intakeIndexerSubsystem)
    );
    addRequirements(intakeIndexerSubsystem, drivebase, shooterSubsystem);
  }
}