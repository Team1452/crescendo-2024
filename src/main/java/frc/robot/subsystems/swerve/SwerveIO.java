package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public class SwerveIO {

  @AutoLog
  public static class SwerveInputs {

    public Pose2d pose = new Pose2d();
  }

  @AutoLog
  public static class ModuleInputs {

    public double driveVelocityMetersPerSecond;
    public double driveVoltage;
    public double driveCurrentAmps;

    public Rotation2d turnPositionRadians;
    public double turnVelocityDegreesPerSecond;
    public double turnVoltage;
    public double turnCurrentAmps;
  }
}
