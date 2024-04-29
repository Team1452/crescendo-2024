package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import java.io.File;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive swerveDrive;
  private final YagslSwerveState swerveInputs = new YagslSwerveState();

  public SwerveSubsystem(File directory) {
    // TODO: Disable in competition.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      swerveDrive =
        new SwerveParser(directory)
          .createSwerveDrive(DriveConstants.kMaxSpeedMetersPerSecond);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    setupPathPlanner();
  }

  public void setupPathPlanner() {
    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        // Translation PID constants:
        new PIDConstants(5.0, 0.0, 0.0),
        // Rotation PID constants:
        new PIDConstants(
          swerveDrive.swerveController.config.headingPIDF.p,
          swerveDrive.swerveController.config.headingPIDF.i,
          swerveDrive.swerveController.config.headingPIDF.d
        ),
        // Max module speed, in m/s:
        DriveConstants.kMaxSpeedMetersPerSecond,
        // Drive base radius in meters. Distance from robot center to furthest module:
        swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
        // Default path replanning config: See the API for the options here.
        new ReplanningConfig()
      ),
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent()
          ? alliance.get() == DriverStation.Alliance.Red
          : false;
      },
      this // Reference to this subsystem to set requirements
    );
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName       PathPlanner path name.
   * @param setOdomToStart Set the odometry position to the start of the path.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName, boolean setOdomToStart) {
    // Load the path you want to follow using its name in the GUI
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    if (setOdomToStart) {
      resetOdometry(new Pose2d(path.getPoint(0).position, getHeading()));
    }

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.followPath(path);
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
      AutoConstants.kMaxSpeedMetersPerSecond,
      DriveConstants.kMaxAccelerationMetersPerSecondSquared,
      0.5 * swerveDrive.getMaximumAngularVelocity(),
      Units.degreesToRadians(720)
    );

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
      pose,
      constraints,
      0.0, // Goal end velocity in meters/sec
      0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(
    DoubleSupplier translationX,
    DoubleSupplier translationY,
    DoubleSupplier headingX,
    DoubleSupplier headingY
  ) {
    // TODO: This may be spurious (https://www.chiefdelphi.com/t/yet-another-generic-swerve-library-yagsl-v1-release/450844/147)
    swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.

    return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth control out
      double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth control out

      driveFieldOriented(
        swerveDrive.swerveController.getTargetSpeeds(
          xInput,
          yInput,
          headingX.getAsDouble(),
          headingY.getAsDouble(),
          swerveDrive.getOdometryHeading().getRadians(),
          swerveDrive.getMaximumVelocity()
        )
      );
    });
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction. Cubed for smoother controls.
   * @param translationY     Translation in the Y direction. Cubed for smoother controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(
    DoubleSupplier translationX,
    DoubleSupplier translationY,
    DoubleSupplier angularRotationX
  ) {
    return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth control out
      double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth control out
      double angularInput = Math.pow(angularRotationX.getAsDouble(), 3); // Smooth control out

      swerveDrive.drive(
        new Translation2d(
          xInput * swerveDrive.getMaximumVelocity(),
          yInput * swerveDrive.getMaximumVelocity()
        ),
        angularInput * swerveDrive.getMaximumAngularVelocity(),
        true,
        false
      );
    });
  }

  /**
   * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
   * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is towards the bow (front) and positive y is
   *                      towards port (left).  In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is towards the left wall when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   */
  public void drive(
    Translation2d translation,
    double rotation,
    boolean fieldRelative
  ) {
    swerveDrive.drive(translation, rotation, fieldRelative, false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  @Override
  public void periodic() {
    // IO should be through an adapter class, but
    // abstraction doesn't really make sense w/ YAGSL.
    // So use AutoLogged class as helper function.
    // TODO: There's probably a nicer way of doing this?
    var state = new YagslModuleState();

    var modules = swerveDrive.getModules();

    for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
      var module = modules[moduleIndex];

      state.fromModule(module);

      Logger.processInputs("Swerve/Module " + (moduleIndex + 1), state);
    }

    swerveInputs.fromYagsl(swerveDrive);

    Logger.processInputs("Swerve", swerveInputs);
    // TODO: will this block the main thread too much?
    // var llresultsArray = new Object[][] {
    //   new Object[] {
    //     "limelight-shooter",
    //     LimelightHelpers.getLatestResults("limelight-shooter"),
    //   },
    //   new Object[] {
    //     "limelight-left",
    //     LimelightHelpers.getLatestResults("limelight-left"),
    //   },
    //   new Object[] {
    //     "limelight-right",
    //     LimelightHelpers.getLatestResults("limelight-right"),
    //   },
    // };
    // int totalAprilTagsSeen = 0;

    // for (Object[] llresultsTuple : llresultsArray) {
    //   String limelightName = (String) llresultsTuple[0];
    //   LimelightResults llresults = (LimelightResults) llresultsTuple[1];

    // int numAprilTagsSeen =
    //   llresults.targetingResults.targets_Fiducials.length;

    // System.out.println(
    //   limelightName + " - AprilTags seen: " + numAprilTagsSeen
    // );

    //   totalAprilTagsSeen += numAprilTagsSeen;

    //   if (numAprilTagsSeen > 0) {
    //     Pose2d proposedPose = llresults.targetingResults.getBotPose2d_wpiBlue();

    //     // Filter out obviously invalid poses (0.0, out of the field, etc.)
    //     if (
    //       proposedPose.getX() == 0.0 ||
    //       proposedPose.getY() == 0.0 ||
    //       proposedPose.getX() < -FieldConstants.kFieldBorderMargin ||
    //       proposedPose.getX() >
    //       (FieldConstants.kFieldLength + FieldConstants.kFieldBorderMargin) ||
    //       proposedPose.getY() < -FieldConstants.kFieldBorderMargin ||
    //       proposedPose.getY() >
    //       (FieldConstants.kFieldWidth + FieldConstants.kFieldBorderMargin)
    //     ) continue;

    //     double poseDifference = swerveDrive
    //       .getPose()
    //       .getTranslation()
    //       .getDistance(proposedPose.getTranslation());

    //     double latency_total_seconds =
    //       (llresults.targetingResults.latency_pipeline / 1000.0) +
    //       (llresults.targetingResults.latency_capture / 1000.0);

    //     double bestTargetArea = Arrays
    //       .stream(llresults.targetingResults.targets_Fiducials)
    //       .mapToDouble(t -> t.ta)
    //       .max()
    //       .orElse(0.0);

    //     double xyStds;
    //     double radStds;
    //     if (numAprilTagsSeen >= 2) {
    //       xyStds = 0.5; // TODO: May need to lower this based on distance? Not trusting enough?
    //       radStds = 0.10472; // ~6 degrees
    //     }
    //     // 1 target with large area and close to estimated pose
    //     else if (bestTargetArea > 0.8 && poseDifference < 0.5) {
    //       xyStds = 1.0;
    //       radStds = 0.20944; // ~12 degrees
    //     }
    //     // 1 target farther away and estimated pose is close
    //     else if (bestTargetArea > 0.1 && poseDifference < 0.3) {
    //       xyStds = 2.0;
    //       radStds = 0.523599; // ~30 degrees
    //     } else continue;

    //     swerveDrive.addVisionMeasurement(
    //       proposedPose,
    //       Timer.getFPGATimestamp() - latency_total_seconds,
    //       VecBuilder.fill(xyStds, xyStds, radStds)
    //     );

    //     System.out.println(
    //       limelightName +
    //       " - Newly Set Pose: |" +
    //       proposedPose +
    //       " Current Timestamp: |" +
    //       Timer.getFPGATimestamp() +
    //       " Latency (s): |" +
    //       latency_total_seconds +
    //       "| Current Pose: |" +
    //       swerveDrive.getPose() +
    //       "|"
    //     );
    //   }
    // }

    // RobotContainer.getInstance().apriltagsSeen.setInteger(totalAprilTagsSeen);
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(
    double xInput,
    double yInput,
    double headingX,
    double headingY
  ) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(
      xInput,
      yInput,
      headingX,
      headingY,
      getHeading().getRadians(),
      DriveConstants.kMaxSpeedMetersPerSecond
    );
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(
    double xInput,
    double yInput,
    Rotation2d angle
  ) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(
      xInput,
      yInput,
      angle.getRadians(),
      getHeading().getRadians(),
      DriveConstants.kMaxSpeedMetersPerSecond
    );
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock() {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }
}
