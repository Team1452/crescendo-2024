package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import java.sql.PseudoColumnUsage;
import java.util.Arrays;

/** An example command that uses an example subsystem. */
public class NoteMode extends Command {

  private final SwerveSubsystem driveSubsystem;

  private final GenericEntry steeringKP = RobotContainer.SHUFFLEBOARD_TAB
    .add("Steering kP", 0.05)
    .getEntry(), steeringKI = RobotContainer.SHUFFLEBOARD_TAB
    .add("Steering kI", 0.01)
    .getEntry(), steeringKD = RobotContainer.SHUFFLEBOARD_TAB
    .add("Steering kD", 0.01)
    .getEntry();

  private final PIDController steeringController = new PIDController(
    0.06,
    0.005,
    0.001
  );


  private final XboxController driverXbox;

  private static double closestTy;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public NoteMode(SwerveSubsystem driveSubsystem, XboxController driverXbox) {
    this.driverXbox = driverXbox;
    this.driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Enabling note mode!");
    // TODO: Update dashboard, etc.

    closestTy = Double.MIN_VALUE;
  }

  private static final double CONFIDENCE_THRESHOLD = 0.5;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    steeringController.setPID(
      steeringKP.getDouble(0),
      steeringKI.getDouble(0),
      steeringKD.getDouble(0)
    );

    var llresults = LimelightHelpers.getLatestResults("limelight-intake");

    double rotationSpeed = 0;

    if (
      llresults != null &&
      llresults.targetingResults.targets_Detector.length > 0
    ) {
      // Sort by largest, and get the first one.
      // There's some built in way to do this
      // with limelight but I don't trust it :)
      var sortedTargets = Arrays
        .stream(llresults.targetingResults.targets_Detector)
        // filter targets below the confidence threshold
        .filter(target -> target.confidence >= CONFIDENCE_THRESHOLD)
        .sorted((a, b) -> Double.compare(b.ta, a.ta))
        .toArray(LimelightTarget_Detector[]::new);

      double contourImageXError = 0;

      if (sortedTargets.length != 0) {
        var closestTarget = sortedTargets[0];

        if (closestTarget.ty_pixels >= closestTy - 5) {
          contourImageXError = closestTarget.tx;
          closestTy = closestTarget.ty_pixels;
        }
        // System.out.println("contour x error " + contourImageXError);
      } else {
        // System.out.println("not seeing any notes in the threshold");
      }

      // TODO: Fine tune, clean up

      rotationSpeed = steeringController.calculate(contourImageXError);
    }

    // System.out.println("Rotation speed: " + rotationSpeed);
    // TODO: Normalize to Swerve speed

    double dr = MathUtil.applyDeadband(-driverXbox.getLeftY(), 0.05) * 3;
    double dtheta = -MathUtil.applyDeadband(driverXbox.getRightX(), 0.05) * 3;

    // +X is robot forward, +Y is robot left
    // System.out.println("NOTE MODE: Driving subsystem");
    driveSubsystem.drive(new Translation2d(dr, dtheta), rotationSpeed, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Exiting note mode!");
    // TODO: Update dashboard, etc.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always running until cancelled
    return false;
  }
}
