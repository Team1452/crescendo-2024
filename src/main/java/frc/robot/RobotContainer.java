package frc.robot;
import frc.robot.commands.RealignToForward;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignToAprilTag;
import frc.robot.commands.AprilTagMode;
import frc.robot.commands.ChaseNote;
import frc.robot.commands.GoToAmp;
import frc.robot.commands.NoteMode;
import frc.robot.commands.autos.BlueShootPreloadAndFetch2;
import frc.robot.commands.autos.RedShootPreloadAndFetch2;
import frc.robot.commands.autos.SpeakerBottomBlueShootPreloadAndFetch;
import frc.robot.commands.autos.SpeakerTopBlueShootPreloadAndFetch;
import frc.robot.commands.autos.SpeakerTopRedShootPreloadAndFetch;
import frc.robot.commands.autos.SquareTest;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeIndexerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

 /*TODO move to it's own subsystem. "drive"*/
public class RobotContainer {

  private static RobotContainer instance = null;

  private final XboxController driverXbox = new XboxController(
    OperatorConstants.kDriverControllerPort
  );

  private final SwerveSubsystem drivebase = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "swerve")
  );
  private final IntakeIndexerSubsystem intakeIndexerSubsystem = new IntakeIndexerSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final LEDSubsystem ledSubsystem = new LEDSubsystem();

  // TODO: Better separation of concern for sharing state between Commands
  public final NoteMode noteMode = new NoteMode(drivebase, driverXbox);
  public double shooterSpeed = 0.2;
  private boolean holdingNote = false;

  public static final ShuffleboardTab SHUFFLEBOARD_TAB = Shuffleboard.getTab(
    "Main"
  );
  private final Field2d field = new Field2d();
  private final ComplexWidget fieldWidget = SHUFFLEBOARD_TAB
    .add("Field", field)
    .withWidget(BuiltInWidgets.kField);
  private final GenericEntry spikeCurrentDifferentialThreshold = SHUFFLEBOARD_TAB
    .add("Spike Current Differential", 1)
    .getEntry();
  private final GenericEntry selectedAuto = SHUFFLEBOARD_TAB
    .add("Selected Auto", "Square Auto")
    .getEntry();

  public void tick() {
    // TODO: This should be in a command (RunCommand isn't executed by scheduler?)
    field.setRobotPose(drivebase.getPose());
  }

  void setHoldingNote(boolean holdingNote) {
    System.out.println("Holding note: " + holdingNote);
    if (holdingNote) {
      driverXbox.setRumble(RumbleType.kBothRumble, 0.05);
      ledSubsystem.setAll(LEDConstants.kColorWhenHoldingNote);
    } else {
      driverXbox.setRumble(RumbleType.kBothRumble, 0);
      ledSubsystem.setAll(LEDConstants.kDefaultColor);
    }
    this.holdingNote = holdingNote;
  }

  Command setHoldingNoteCommand(boolean holdingNote) {
    return new InstantCommand(() -> setHoldingNote(holdingNote));
  }

  double getShooterSpeed() {
    return shooterSpeed;
  }

  public static RobotContainer getInstance() {
    return instance;
  }

  public RobotContainer() {
    instance = this;

    /*************************************************************************/

    driverXbox.setRumble(RumbleType.kBothRumble, 0);

    // TODO: Document why everything is inverted, and the new coord system.

    // TODO: This should be better organized/gains shared properly
    final PIDController degreeAngleController = new PIDController(
      0.016,
      0.002,
      0.001
    );

    SHUFFLEBOARD_TAB.add("Degree Controller", degreeAngleController);

    drivebase.setDefaultCommand(
      drivebase.driveCommand(
        () ->
          MathUtil.applyDeadband(
            -driverXbox.getLeftY(), // Inverted.
            OperatorConstants.kTranslationDeadband
          ) *
          (1 - driverXbox.getRightTriggerAxis()), // remove
        () ->
          MathUtil.applyDeadband(
            -driverXbox.getLeftX(), // Inverted.
            OperatorConstants.kTranslationDeadband
          ) *
          (1 - driverXbox.getRightTriggerAxis()
          ),
        () -> {
          var pov = driverXbox.getPOV();

          if (pov != -1) {
            // Angle control to increment of 45 degrees
            System.out.println("POV: " + pov);
            double targetAngle = pov;
            double currentAngle = drivebase
              .getPose()
              .getRotation()
              .getDegrees();
            double angleDifference = targetAngle - currentAngle;
            if (angleDifference > 180) {
              angleDifference -= 360;
            } else if (angleDifference < -180) {
              angleDifference += 360;
            }
            return degreeAngleController.calculate(angleDifference);
          }

          double rightX = MathUtil.applyDeadband(
            -driverXbox.getRightX(),
            OperatorConstants.kTranslationDeadband
          );

          return rightX;
        }
      )
    );

    /*********************** NAMED COMMANDS ************************** */

    NamedCommands.registerCommand(
      "ConsumeNote",
      intakeIndexerSubsystem
        .createIntakeNoteCommand()
        .raceWith(new ChaseNote(drivebase))
        .withTimeout(5000)
    );

    NamedCommands.registerCommand(
      "AlignToAprilTag",
      new AlignToAprilTag(drivebase)
    );

    NamedCommands.registerCommand(
      "AlignAndShoot",
      new AlignToAprilTag(drivebase)
        .withTimeout(1000)
        .andThen(
          shooterSubsystem
            .createShootSpeakerCommand(intakeIndexerSubsystem)
            .handleInterrupt(() -> shooterSubsystem.setShooter(0))
        )
    );

    /*************************************************************** */

    // Y button will run shoot routine.
    // TODO: Only run command if note is currently held? (State managed by IntakeNode)
    new Trigger(driverXbox::getYButton)
      .toggleOnTrue(
        setHoldingNoteCommand(false)
          .andThen(
            shooterSubsystem
              .createShootSpeakerCommand(intakeIndexerSubsystem)
              .handleInterrupt(() -> shooterSubsystem.setShooter(0))
          )
      );

    // X button shoots for Amp (tentative)
    new Trigger(driverXbox::getXButton)
      .toggleOnTrue(
        setHoldingNoteCommand(false)
          .andThen(
            shooterSubsystem.createShootAmpCommand(intakeIndexerSubsystem)
          )
      );
    new Trigger(driverXbox::getLeftStickButton) //realign to yaw 0 on stick press
      .toggleOnTrue(
        new RealignToForward(drivebase)
          .handleInterrupt(() -> drivebase.drive(new Translation2d(0, 0), 0, true))
      );

    /* CLIMB LOGIC */
    var climbSubsystem = new ClimbSubsystem();

    // If right stick button is pressed, enter climb mode.s
    // Left/right hooks are controlled like tank drive,
    // robot only moves forward/backward.
    var climbMode = new InstantCommand(() ->
      ledSubsystem.setAll(LEDConstants.kColorWhenClimbing)
    )
      .andThen(
        new RunCommand(
          () -> {
            double leftClimbSpeed = -driverXbox.getLeftTriggerAxis();
            double rightClimbSpeed = -driverXbox.getRightTriggerAxis();

            double leftClimbVelocity =
              (driverXbox.getLeftBumper() ? 1 : -1) * leftClimbSpeed;
            double rightClimbVelocity =
              (driverXbox.getRightBumper() ? 1 : -1) * rightClimbSpeed;

            System.out.println(
              "Left: " +
              climbSubsystem.getLeftPosition() +
              ", Right: " +
              climbSubsystem.getRightPosition()
            );

            if (
              leftClimbVelocity < 0 &&
              (
                climbSubsystem.leftLimitHit() ||
                climbSubsystem.getLeftPosition() < 1e-3
              )
            ) leftClimbVelocity = 0;
            if (
              rightClimbVelocity < 0 && 
              (
                climbSubsystem.rightLimitHit() ||
                climbSubsystem.getRightPosition() < 1e-3
              )
            ) rightClimbVelocity = 0;

            climbSubsystem.setClimbOutputs(
              leftClimbVelocity,
              rightClimbVelocity
            );

            double velocity =
              -MathUtil.applyDeadband(driverXbox.getLeftY(), 0.05) * 2;

            // Drive forward/backward
            drivebase.drive(
              new Translation2d(velocity, 0),
              driverXbox.getRightX(),
              true
            );
          },
          drivebase,
          climbSubsystem,
          intakeIndexerSubsystem
        ) 
      )
      .beforeStarting(() -> {
        System.out.println("Entering climb mode...");
      })
      .andThen(() -> {
        ledSubsystem.setAll(LEDConstants.kDefaultColor);
        System.out.println("Exiting climb mode...");
      })
      .handleInterrupt(() -> {
        climbSubsystem.setClimbOutputs(0, 0);
      });

    // "Climb mode" toggled w/ right stick button
    new Trigger(() -> driverXbox.getRightStickButtonPressed())
      .toggleOnTrue(climbMode);

    // While left bumper is held, align to nearest AprilTag and control relative to AprilTag
    new Trigger(() -> driverXbox.getLeftBumper() && !climbMode.isScheduled())
      .whileTrue(new AprilTagMode(drivebase, driverXbox));

    // B button will outtake note
    new Trigger(() -> driverXbox.getBButton() && !climbMode.isScheduled())
      .whileTrue(
        setHoldingNoteCommand(false)
          .andThen(intakeIndexerSubsystem.createOuttakeNoteCommand())
      );

    // While right bumper is held, align to closest note and control relative to note
    new Trigger(() -> driverXbox.getRightBumper() && !climbMode.isScheduled())
      .whileTrue(
        noteMode.alongWith(
          intakeIndexerSubsystem
            .createIntakeNoteCommand()
            .andThen(setHoldingNoteCommand(true))
        )
      );

    new Trigger(() -> driverXbox.getRawButton(7) && !climbMode.isScheduled())
      .whileTrue(new GoToAmp(drivebase));

    
  
    // Pull down. Pull left until current limit, pull right until current limit, then pull both.
    // new Trigger(() -> driverXbox.getLeftBumper())
    //   .toggleOnTrue(new SequentialCommandGroup(
    //   // Pull left down until soft limit or current threshold
    //   new RunCommand(() -> {
    //     double leftTrigger = driverXbox.getLeftTriggerAxis();
    //     climbSubsystem.setClimbOutputsDown(leftTrigger, 0);
    //   }, climbSubsystem)
    //     .onlyWhile(() -> climbSubsystem.getLeftClimbCurrent() - climbSubsystem.getLeftClimbCurrent()
    //       < spikeCurrentDifferentialThreshold.getDouble(0)
    //       && climbSubsystem.getRightPosition() > 0)
    //     .andThen(() -> climbSubsystem.setClimbOutputsDown(0, 0)),

    //   // Pull right down until soft limit or current threshold
    //   new RunCommand(() -> {
    //     double leftTrigger = driverXbox.getLeftTriggerAxis();
    //     climbSubsystem.setClimbOutputsDown(0, leftTrigger);
    //   }, climbSubsystem)
    //     .onlyWhile(() -> climbSubsystem.getRightClimbCurrent() - climbSubsystem.getLastRightClimbCurrent()
    //         < spikeCurrentDifferentialThreshold.getDouble(0)
    //       && climbSubsystem.getRightPosition() > 0)
    //     .andThen(() -> climbSubsystem.setClimbOutputsDown(0, 0)),

    //   // Pull both down according to controller
    //   new RunCommand(() -> {
    //     double leftTrigger = driverXbox.getLeftTriggerAxis();
    //     climbSubsystem.setClimbOutputsDown(leftTrigger, leftTrigger);
    //   })
    // ));

    // // Extend. Don't worry about current (rotation soft limits)
    // new Trigger(() -> !driverXbox.getLeftBumper())
    //   .toggleOnTrue(new RunCommand(() -> {
    //     double climbSpeed = driverXbox.getLeftTriggerAxis();
    //     climbSubsystem.setClimbOutputsUp(climbSpeed, climbSpeed);
    //   }, climbSubsystem));

    /*************************************{{{************************************/

    // Key: https://joytokey.net/en/posts/button-mapping-for-xbox-controller

    new JoystickButton(driverXbox, 1)
      .onTrue((new InstantCommand(drivebase::zeroGyro))); // A
    // new JoystickButton(driverXbox, 3)
    // .whileTrue(
    // new RepeatCommand(new InstantCommand(drivebase::lock, drivebase))
    // );

    /*************************************************************************/
  }

  public Command getAutonomousCommand() {
    return new RedShootPreloadAndFetch2(
      intakeIndexerSubsystem,
      shooterSubsystem,
      drivebase
    );
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public void zeroGyro() {
    drivebase.zeroGyro();
  }
}
