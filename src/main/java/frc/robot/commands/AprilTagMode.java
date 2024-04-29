package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class AprilTagMode extends Command {

  private final SwerveSubsystem driveSubsystem;
  private final PIDController steeringController = new PIDController(
    0.03,
    0.001,
    0.001
  );
  private final double speakerFiducialId = DriverStation
      .getAlliance()
      .orElseGet(() -> Alliance.Blue) ==
    Alliance.Red
    ? 4.0 // Centered tag for red.
    : 7.0; // Centered tag for blue.
  private final double ampFiducialId = DriverStation
      .getAlliance()
      .orElseGet(() -> Alliance.Blue) ==
    Alliance.Red
    ? 5.0 // Centered tag for red.
    : 6.0; // Centered tag for blue.

  private final XboxController driverXbox;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AprilTagMode(
    SwerveSubsystem driveSubsystem,
    XboxController driverXbox
  ) {
    this.driverXbox = driverXbox;
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
    double leftY = MathUtil.applyDeadband(-driverXbox.getLeftY(), 0.05);
    double velocity = leftY;

    var llresults = LimelightHelpers.getLatestResults("limelight-shooter");

    // 42 inches away, targeting 8 inches in front of apriltag in robot space
    // TODO: Use this ^^^

    for (LimelightTarget_Fiducial target : llresults.targetingResults.targets_Fiducials) {
      if (
        target.fiducialID == speakerFiducialId ||
        target.fiducialID == ampFiducialId
      ) {
        double tx = target.tx; // Offset angle.

        Pose2d fieldPose2d = target.getRobotPose_FieldSpace2D();
        Pose2d targetPoseRobotSpace = target.getTargetPose_RobotSpace2D();

        System.out.println(
          "target pose: " +
          targetPoseRobotSpace +
          ". Distance: " +
          Math.hypot(targetPoseRobotSpace.getX(), targetPoseRobotSpace.getY())
        );
        double rotationSpeed = steeringController.calculate(tx);

        // TODO: this should be dynamic based on distance
        if (Math.abs(tx) < 1.5) {
          System.out.println(
            "X: " +
            target.getTargetPose_RobotSpace2D().getX() +
            " Y:" +
            target.getTargetPose_RobotSpace2D().getY()
          );

          System.out.println("Aligned! READY TO FIRE!!!!!!!!!!!!!!!!");
          // aligned = true;
        }

        // +X is robot forward, +Y is robot left
        driveSubsystem.drive(
          new Translation2d(velocity, 0),
          rotationSpeed,
          false
        );

        break;
      }
    }
    // TODO? Don't do shit if we don't see a tag?
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
