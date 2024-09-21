package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.Constants.*;
public class RealignToForward extends Command {
    private final SwerveSubsystem drivebase;
    private final PIDController turnController;
    private static final double kP = 0.01; // Proportional control gain, tune as necessary
    private static final double kI = 0.0;  // Integral control gain, set to 0 for now
    private static final double kD = 0.0;  // Derivative control gain, set to 0 for now
    private static final double TOLERANCE_DEGREES = 1.0; // Acceptable tolerance for being aligned

    public RealignToForward(SwerveSubsystem drivebase) {
        this.drivebase = drivebase;

        // Initialize PID controller for rotational control
        turnController = new PIDController(DriveConstants.kAutoSteeringP, DriveConstants.kAutoSteeringI, DriveConstants.kAutoSteeringD);
        turnController.setTolerance(TOLERANCE_DEGREES); // Set tolerance for alignment
        turnController.enableContinuousInput(-180.0, 180.0); // Handle wraparound at -180 and 180 degrees

        addRequirements(drivebase); // Declare subsystem dependencies
    }

    @Override
    public void initialize() {
        // Set target angle to 0 (forward)
        turnController.setSetpoint(0.0);
    }


    @Override
    public void execute() {
        // Get the current robot heading from the gyro
        double currentHeading = drivebase.getYaw();
        
        // Calculate the rotation power needed to reach the target
        double rotationSpeed = turnController.calculate(currentHeading);
        
        // Apply the rotation speed to the robot2
        drivebase.drive(new Translation2d(0, 0), rotationSpeed, true); // Only rotate, no translation
    }

    @Override
    public boolean isFinished() {
        // Stop when the robot is within the acceptable tolerance of facing forward (0 degrees)
        return turnController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when done
        drivebase.drive(new Translation2d(0, 0), 0, true);
    }
}