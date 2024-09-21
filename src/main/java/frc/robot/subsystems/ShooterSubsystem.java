package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CANIds;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ShooterSubsystem extends SubsystemBase {

  private final double targetFiducialId = DriverStation
      .getAlliance()
      .orElseGet(() -> Alliance.Blue) ==
    Alliance.Red
    ? 4.0 // Centered tag for red.
    : 7.0; // Centered tag for blue.
  private boolean seesSpeakerAprilTag = false;

  private final Timer seenSpeakerTimer = new Timer();

  private CANSparkMax leftShooterMotor, rightShooterMotor; 
  private GenericEntry ampShooterSpeed = RobotContainer.SHUFFLEBOARD_TAB
    .add("Amp Speed", 0.15)
    .getEntry();
  private GenericEntry speakerShooterSpeed = RobotContainer.SHUFFLEBOARD_TAB
    .add("Speaker Speed", 1)
    .getEntry();

  public ShooterSubsystem() {
    leftShooterMotor = new CANSparkMax(CANIds.kLeftShooterMotor, MotorType.kBrushless); 
    rightShooterMotor = new CANSparkMax(CANIds.kRightShooterMotor,MotorType.kBrushless); 
    // Invert left shooter motor, keep right the same
    leftShooterMotor.setInverted(true);
    rightShooterMotor.setInverted(false);
  }

  public void setShooter(double velocity) {
    leftShooterMotor.set(velocity);
    rightShooterMotor.set(velocity);
  }

  @Override
  public void periodic() {
    // Consume drivebase, meaning drivebase's default command (swerve drive) is disabled.
    var llresults = LimelightHelpers.getLatestResults("limelight-shooter");

    for (LimelightTarget_Fiducial target : llresults.targetingResults.targets_Fiducials) {
      // System.out.println(
      //   "Found " + target.fiducialID + ". Target: " + targetFiducialId
      // );
      // Driver Station wasn't returning team?
      if (target.fiducialID == 4 || target.fiducialID == 7) {
        // System.out.println("Found speaker, spinning up shooter");

        setShooter(0.30);
        seenSpeakerTimer.reset();
        seenSpeakerTimer.start();
        seesSpeakerAprilTag = true;
        return;
      }
    }

    seesSpeakerAprilTag = false;
    // System.out.println("Can't see speaker");

    if (seenSpeakerTimer.hasElapsed(0.5)) {
      // System.out.println(
      //   "Speaker not seen for 0.5 seconds, setting shooter to 0."
      // );

      setShooter(0);
      seenSpeakerTimer.reset();
      seenSpeakerTimer.stop();
    }
  }

  public Command createShootAmpCommand(
    IntakeIndexerSubsystem intakeIndexerSubsystem
  ) {
    return new SequentialCommandGroup(
      runOnce(() -> setShooter(ampShooterSpeed.getDouble(0))),
      new WaitCommand(0.4),
      runOnce(() -> intakeIndexerSubsystem.setIndexer(1)),
      new WaitCommand(0.4),
      runOnce(() -> {
        intakeIndexerSubsystem.setIndexer(0);
        setShooter(0);
      })
    )
      .handleInterrupt(() -> {
        intakeIndexerSubsystem.setIndexer(0);
        setShooter(0);
      });
  }

  public Command createShootSpeakerCommand(
    IntakeIndexerSubsystem intakeIndexerSubsystem
  ) {
    return new SequentialCommandGroup(
      runOnce(() -> {
        setShooter(speakerShooterSpeed.getDouble(0));
      }),
      new WaitCommand(seesSpeakerAprilTag ? 0 : 0.4),
      runOnce(() -> intakeIndexerSubsystem.setIndexer(1)),
      new WaitCommand(0.4),
      runOnce(() -> {
        intakeIndexerSubsystem.setIndexer(0);
        setShooter(0);
      })
    )
      .handleInterrupt(() -> {
        intakeIndexerSubsystem.setIndexer(0);
        setShooter(0);
      });
  }

  public Command createShootAmpCommandAndMoveBack(
    IntakeIndexerSubsystem intakeIndexerSubsystem,
    SwerveSubsystem drivebase
  ) {
    var cmd = new SequentialCommandGroup(
      createShootAmpCommand(intakeIndexerSubsystem),
      runOnce(() -> drivebase.drive(new Translation2d(1, 0), 0, false)),
      new WaitCommand(0.3),
      runOnce(() -> drivebase.drive(new Translation2d(), 0, false))
    );
    cmd.addRequirements(intakeIndexerSubsystem, drivebase);
    return cmd;
  }
}
