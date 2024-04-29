package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.ClimbConstants;

// Wrapper around climb motors
public class ClimbSubsystem extends SubsystemBase {

  private CANSparkMax leftClimber, rightClimber;
  private DigitalInput leftLimitSwitch = new DigitalInput(
    5
  ), rightLimitSwitch = new DigitalInput(6);

  public ClimbSubsystem() {
    leftClimber = new CANSparkMax(CANIds.kClimberLeft, MotorType.kBrushless);
    rightClimber = new CANSparkMax(CANIds.kClimberRight, MotorType.kBrushless);

    leftClimber.restoreFactoryDefaults();
    rightClimber.restoreFactoryDefaults();

    leftClimber.setIdleMode(IdleMode.kBrake);
    rightClimber.setIdleMode(IdleMode.kBrake);

    // leftClimber.setSoftLimit(SoftLimitDirection.kReverse, 0);
    // rightClimber.setSoftLimit(SoftLimitDirection.kReverse, 0);

    leftClimber.getEncoder().setPosition(0);
    rightClimber.getEncoder().setPosition(0);

    // TODO: set kForward soft limit in rotations
    // leftClimber.setSoftLimit(SoftLimitDirection.kForward, ClimbConstants.kForwardLimitRotations);
    // rightClimber.setSoftLimit(SoftLimitDirection.kForward, ClimbConstants.kForwardLimitRotations);

    // Maybe use PIDs? Use voltage control for now
    // leftClimberPositionController = leftClimber.getPIDController();
    // rightClimberPositionController = rightClimber.getPIDController();

    // setPID(kClimbP, kClimbI, kClimbD);

    rightClimber.setInverted(false);
    leftClimber.setInverted(false);
  }

  public void reset() {
    leftClimber.getEncoder().setPosition(0);
    rightClimber.getEncoder().setPosition(0);
  }

  public void setPID(double kP, double kI, double kD) {
    // leftClimberPositionController.setP(kClimbP);
    // leftClimberPositionController.setI(kClimbI);
    // leftClimberPositionController.setD(kClimbI);

    // rightClimberPositionController.setP(kClimbP);
    // rightClimberPositionController.setI(kClimbI);
    // rightClimberPositionController.setD(kClimbI);
  }

  public void setTargetPositions(
    double leftHookPosition,
    double rightHookPosition
  ) {
    // double clampedLeftHookPosition = MathUtil.clamp(leftHookPosition, 0, kFurthestPositionMeters),
    //     clampedRightHookPosition = MathUtil.clamp(rightHookPosition, 0, kFurthestPositionMeters);

    // leftClimberPositionController.setReference(clampedLeftHookPosition, ControlType.kPosition);
    // rightClimberPositionController.setReference(clampedRightHookPosition, ControlType.kPosition);
  }

  // Rotations for now. TODO: Meters?
  public double getLeftPosition() {
    return leftClimber.getEncoder().getPosition();
  }

  public double getRightPosition() {
    return rightClimber.getEncoder().getPosition();
  }

  public boolean leftLimitHit() {
    return !leftLimitSwitch.get();
  }

  public boolean rightLimitHit() {
    return !rightLimitSwitch.get();
  }

  public void setClimbOutputs(
    double leftClimbVelocity,
    double rightClimbVelocity
  ) {
    if (
      (leftClimbVelocity < 0 && leftClimber.getEncoder().getPosition() < 0) ||
      (
        leftClimbVelocity > 0 &&
        leftClimber.getEncoder().getPosition() >=
        ClimbConstants.kLeftUpperLimitRotations
      )
    ) leftClimber.set(0); else leftClimber.set(leftClimbVelocity);

    if (
      rightClimbVelocity < 0 &&
      rightClimber.getEncoder().getPosition() < 0 ||
      (
        rightClimbVelocity > 0 &&
        rightClimber.getEncoder().getPosition() >=
        ClimbConstants.kRightUpperLimitRotations
      )
    ) rightClimber.set(0); else rightClimber.set(rightClimbVelocity);
  }

  double lastLeftClimbCurrent, lastRightClimbCurrent;

  public double getLastLeftClimbCurrent() {
    return lastLeftClimbCurrent;
  }

  public double getLastRightClimbCurrent() {
    return lastRightClimbCurrent;
  }

  public double getLeftClimbCurrent() {
    double current = leftClimber.getOutputCurrent();
    lastLeftClimbCurrent = current;
    return current;
  }

  public double getRightClimbCurrent() {
    double current = rightClimber.getOutputCurrent();
    lastRightClimbCurrent = current;
    return current;
  }

  @Override
  public void periodic() {}
}
