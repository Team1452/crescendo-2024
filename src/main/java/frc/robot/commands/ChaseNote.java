package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.Arrays;

/** An example command that uses an example subsystem. */
public class ChaseNote extends Command {

  private final SwerveSubsystem driveSubsystem;
  private final PIDController steeringController = new PIDController(
    0.03,
    0.001,
    0.001
  );

  private static final double CONFIDENCE_THRESHOLD = 0.5;

  private static double closestTy;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ChaseNote(SwerveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    closestTy = Double.NEGATIVE_INFINITY;
    // TODO: Update dashboard, etc.

    closestTy = Double.MIN_VALUE;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var llresults = LimelightHelpers.getLatestResults("limelight-intake");

    double rotationSpeed = 0;

    if (
      llresults != null &&
      llresults.targetingResults.targets_Detector.length > 0
    ) {
      // Sort by largest, and get the first one.
      // There's some built in way to do this
      // with limelight but I don't trust it :)
      LimelightTarget_Detector[] sortedTargets = Arrays
        .stream(llresults.targetingResults.targets_Detector)
        // filter targets below the confidence threshold
        .filter(target -> target.confidence >= CONFIDENCE_THRESHOLD)
        .sorted((a, b) -> Double.compare(b.ta, a.ta))
        .toArray(LimelightTarget_Detector[]::new);

      if (sortedTargets.length == 0) {
        System.out.println("no more targets");
        // TODO: do we need to do this, or can we just do nothing?
        driveSubsystem.drive(new Translation2d(1, 0), 0, false);

        return;
      }

      var closestTarget = sortedTargets[0];

      if (sortedTargets.length > 1) {
        // if the top two tas are both within 10% (percent difference) of each other,
        // use the one with a smaller tx
        // gas algorithim courtesy of lany hill the third
        if (
          Math.abs(sortedTargets[0].ta - sortedTargets[1].ta) /
          Math.abs(((sortedTargets[0].ta + sortedTargets[1].ta) / 2)) <
          0.1
        ) {
          closestTarget =
            Math.abs(sortedTargets[0].tx) < Math.abs(sortedTargets[1].tx)
              ? sortedTargets[0]
              : sortedTargets[1];
        }
      }

      double contourImageXError = closestTarget.tx;

      if (closestTarget.ty_pixels >= closestTy - 5) {
        // System.out.println("contour x error " + contourImageXError);

        closestTy = closestTarget.ty_pixels;

        // TODO: Fine tune, clean up

        rotationSpeed = steeringController.calculate(contourImageXError);

        // +X is robot forward, +Y is robot left
        // TODO: Is this an issue? ^^^
        driveSubsystem.drive(
          new Translation2d(1/* 0.5 m/s forward */, 0),
          rotationSpeed,
          false
        );
        // System.out.println("closest target ty: " + closestTarget.ty);
      } else {
        // System.out.println("rejected because ty is of " + closestTarget.ty);

        driveSubsystem.drive(new Translation2d(1/* 1.5 m/s */, 0), 0, false);
      }
    } else {
      // TODO: do we need to do this, or can we just do nothing?
      driveSubsystem.drive(new Translation2d(1/* 1.5 m/s */, 0), 0, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // TODO: Update dashboard, etc.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
