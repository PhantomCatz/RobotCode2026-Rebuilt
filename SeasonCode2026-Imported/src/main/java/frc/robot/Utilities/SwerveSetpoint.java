package frc.robot.Utilities;

import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

public record SwerveSetpoint(ChassisVelocities ChassisVelocities, SwerveModuleVelocity[] moduleStates) {
}
