package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeIndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AlignAndShoot extends SequentialCommandGroup {

  public AlignAndShoot(
    SwerveSubsystem drivebase,
    ShooterSubsystem shooterSubsystem,
    IntakeIndexerSubsystem intakeIndexerSubsystem
  ) {
    addCommands(
      new AlignToAprilTag(drivebase)
        .withTimeout(
          0
        )/* Don't need to align for Auto. Ideally this would terminate on its own */
        .andThen(
          shooterSubsystem
            .createShootSpeakerCommand(intakeIndexerSubsystem)
            .handleInterrupt(() -> shooterSubsystem.setShooter(0))
        )
    );
  }
}
