package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class AlignToAprilTag extends Command {

  private final SwerveSubsystem driveSubsystem;
  private final PIDController steeringController = new PIDController(
    0.03,
    0.001,
    0.001
  );
  private final double targetFiducialId = DriverStation
      .getAlliance()
      .orElseGet(() -> Alliance.Blue) ==
    Alliance.Red
    ? 4.0 // Centered tag for red.
    : 7.0; // Centered tag for blue.
  private boolean aligned = false;

  public AlignToAprilTag(SwerveSubsystem driveSubsystem) {
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
    // Consume drivebase, meaning drivebase's default command (swerve drive) is disabled.
    var llresults = LimelightHelpers.getLatestResults("limelight-shooter");

    for (LimelightTarget_Fiducial target : llresults.targetingResults.targets_Fiducials) {
      if (target.fiducialID == 4 || target.fiducialID == 7) {
        double tx = target.tx; // Offset angle.

        System.out.println("tx: " + tx);
        double rotationSpeed = steeringController.calculate(tx);

        // TODO: this should be dynamic based on distance
        if (Math.abs(tx) < 1.5) {
          System.out.println(
            "X: " +
            target.getTargetPose_RobotSpace2D().getX() +
            " Y:" +
            target.getTargetPose_RobotSpace2D().getY()
          );

          aligned = true;
        }

        driveSubsystem.drive(new Translation2d(), rotationSpeed, true);

        break;
      }
    }

    // Don't do shit if we don't see a tag.
    driveSubsystem.drive(new Translation2d(), 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return aligned;
  }
}
