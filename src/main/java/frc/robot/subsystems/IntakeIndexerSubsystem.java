package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.DIOPorts;

public class IntakeIndexerSubsystem extends SubsystemBase {

  private CANSparkMax indexerMotor, intakeMotor;
  private DigitalInput breakBeam = new DigitalInput(
    DIOPorts.kIntakeFrontBreakBeam
  );

  private boolean beamWasBrokenLastTick = false;

  public IntakeIndexerSubsystem() {
    indexerMotor = new CANSparkMax(CANIds.kIndexerMotor, MotorType.kBrushed);
    intakeMotor = new CANSparkMax(CANIds.kIntakeMotor, MotorType.kBrushed);
  }

  @Override
  public void periodic() {
    // System.out.println("Break beam: " + isBeamBroken());
  }

  public void setIntake(double velocity) {
    intakeMotor.set(velocity);
  }

  public void setIndexer(double velocity) {
    indexerMotor.set(velocity);
  }

  public boolean wasBeamBrokenLastTick() {
    return beamWasBrokenLastTick;
  }

  public Command createOuttakeNoteCommand() {
    return new RunCommand(() -> {
      // Start intake, stop indexer
      System.out.println("Running outtake");
      setIntake(-0.5);
      setIndexer(-0.5);
    })
      .handleInterrupt(() -> {
        // Clean up
        setIntake(0);
        setIndexer(0);
      });
  }

  public Command createIntakeNoteCommand() {
    return new SequentialCommandGroup(
      // Start intake, stop indexer
      runOnce(() -> {
        setIntake(0.75);
        setIndexer(0.5);
      }),
      // Once beam is broken, run intake and indexer slowly
      new InstantCommand(() -> System.out.println("Waiting for beam")),
      new WaitUntilCommand(() -> isBeamBroken()),
      new InstantCommand(() ->
        System.out.println("Beam broken, waiting for cross")
      ),
      runOnce(() -> {
        setIntake(0.2);
        setIndexer(0.2);
      }),
      // Once note crossed beam (it was broken and now isn't), stop both motors and exit
      new WaitUntilCommand(() -> wasBeamBrokenLastTick() && !isBeamBroken()),
      new InstantCommand(() ->
        System.out.println("Beam crossed! Ending command")
      ),
      runOnce(() -> {
        setIntake(0);
        setIndexer(0);
      })
    )
      .handleInterrupt(() -> {
        // Clean up
        setIntake(0);
        setIndexer(0);
      });
  }

  public boolean isBeamBroken() {
    // TODO: Normally closed or normally open?
    boolean beamBroken = !breakBeam.get();
    beamWasBrokenLastTick = beamBroken;
    return beamBroken;
  }
}
