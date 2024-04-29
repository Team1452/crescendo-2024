// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import swervelib.SwerveModule;

public class YagslModuleState extends ModuleInputsAutoLogged {

  public void fromModule(SwerveModule module) {
    // Get SparkMaxes directly for current output reading
    var driveMotor = (CANSparkMax) module.configuration.driveMotor.getMotor();
    var angleMotor = (CANSparkMax) module.configuration.angleMotor.getMotor();

    driveVelocityMetersPerSecond = module.getDriveMotor().getVelocity();
    driveVoltage = module.getDriveMotor().getVoltage();
    driveCurrentAmps = driveMotor.getOutputCurrent();

    turnPositionRadians = Rotation2d.fromDegrees(module.getAbsolutePosition());
    turnVelocityDegreesPerSecond = module.getAngleMotor().getVelocity();
    turnVoltage = module.getAngleMotor().getVoltage();
    turnCurrentAmps = angleMotor.getOutputCurrent();
  }
}
