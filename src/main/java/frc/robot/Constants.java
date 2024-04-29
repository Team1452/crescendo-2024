package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class CANIds {

    /* CAN IDs for drive and steering motors, cancoders and pigeon are managed by YAGSL */

    public static final int kLeftShooterMotor = 5;
    public static final int kRightShooterMotor = 6;

    public static final int kIndexerMotor = 7;
    public static final int kIntakeMotor = 8;

    // "Left" and "right" relative to intake side as front
    public static final int kClimberLeft = 9;
    public static final int kClimberRight = 19;
  }

  public static class DIOPorts {

    public static final int kIntakeFrontBreakBeam = 9;
  }

  public static class ClimbConstants {

    public static final double kRightUpperLimitRotations = 284.5;
    public static final double kLeftUpperLimitRotations = 344.9;
  }

  public static class LEDConstants {

    public static final int kPWMPort = 3;
    public static final int kLength = 74;
    public static final Color kColorWhenHoldingNote = Color.kGreen;
    public static final Color kDefaultColor = Color.kRed;
    public static final Color kDisableColor = Color.kDodgerBlue;
    public static final Color kColorWhenClimbing = Color.kPurple;
  }

  public static class DriveConstants {

    public static final double kMaxSpeedMetersPerSecond = 4.0; // TODO: Determine max possible/attainable speed
    public static final double kMaxAccelerationMetersPerSecondSquared = 4.0; // TODO: Determine max possible/attainable acceleration

    public static final double kAutoSteeringP = 0.05;
    public static final double kAutoSteeringI = 0.02;
    public static final double kAutoSteeringD = 0.001;
  }

  public static class FieldConstants {

    // From https://github.com/Mechanical-Advantage/RobotCode2024
    public static final double kFieldBorderMargin = 0.5;
    public static final double kFieldLength = Units.inchesToMeters(651.223);
    public static final double kFieldWidth = Units.inchesToMeters(323.277);
  }

  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;
    public static final int kMechControllerPort = 1;
    public static final double kTranslationDeadband = 0.035;
  }

  public static class AutoConstants {

    public static final double kMaxSpeedMetersPerSecond = 1.5;
  }
}
