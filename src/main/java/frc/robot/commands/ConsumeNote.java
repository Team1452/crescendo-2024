package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeIndexerSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ConsumeNote extends SequentialCommandGroup {

  public ConsumeNote(
    SwerveSubsystem drivebase,
    IntakeIndexerSubsystem intakeIndexerSubsystem
  ) {
    addCommands(
      intakeIndexerSubsystem
        .createIntakeNoteCommand()
        .raceWith(new ChaseNote(drivebase))
    );
  }
}
