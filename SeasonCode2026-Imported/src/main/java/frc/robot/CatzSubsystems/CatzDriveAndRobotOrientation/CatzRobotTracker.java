package frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation;

import org.wpilib.math.geometry.*;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants;

import java.util.function.Supplier;

import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;


public class CatzRobotTracker {
  public static CatzRobotTracker Instance;

  // ------------------------------------------------------------------------------------------------------
  //  Pose estimation Members
  // ------------------------------------------------------------------------------------------------------
  @Getter
  @AutoLogOutput(key = "CatzRobotTracker/PureOdometryPose")
  private Pose2d odometryPose = new Pose2d();

  @Getter
  private Pose2d estimatedPose = new Pose2d(8.0, 4.0, new Rotation2d());

  // Odometry
  private final SwerveDriveKinematics KINEMATICS;
  private SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  @Getter
  private SwerveModuleVelocity[] currentModuleStates =
      new SwerveModuleVelocity[] {
        new SwerveModuleVelocity(),
        new SwerveModuleVelocity(),
        new SwerveModuleVelocity(),
        new SwerveModuleVelocity()
      };

  private Rotation2d lastGyroAngle = new Rotation2d();
  private Twist2d robotAccelerations = new Twist2d();
  private ChassisVelocities m_lastChassisVelocities = new ChassisVelocities();
  private double lastTimestamp = 0.0;

  // ------------------------------------------------------------------------------------------------------
  //
  //  Constructor
  //
  // ------------------------------------------------------------------------------------------------------
  private CatzRobotTracker() {
    KINEMATICS = DriveConstants.SWERVE_KINEMATICS;
  }

  // ------------------------------------------------------------------------------------------------------
  //
  //  Pose Estimation adder methods
  //
  // ------------------------------------------------------------------------------------------------------
  /** Add odometry observation */
  public void addOdometryObservation(OdometryObservation observation) {
    currentModuleStates = observation.moduleStates;
    // Calculate twist from last wheel positions to current wheel positions, useful for when gyro is
    // disabled
    Twist2d twist = KINEMATICS.toTwist2d(lastWheelPositions, observation.wheelPositions());
    lastWheelPositions = observation.wheelPositions();
    //Check gyro connected
    if (observation.gyroAngle != null) {
      // Update dtheta for twist if gyro connected
      twist =
          new Twist2d(
              twist.dx, twist.dy, observation.gyroAngle().minus(lastGyroAngle).getRadians());
      lastGyroAngle = observation.gyroAngle();
    }

    //Add twist to odometry pose
    if((twist.dx != 0 || twist.dy != 0 || twist.dtheta != 0) && (!Double.isNaN(twist.dx) && !Double.isNaN(twist.dy) && !Double.isNaN(twist.dtheta))){
      odometryPose = odometryPose.plus(twist.exp());
      estimatedPose = estimatedPose.plus(twist.exp());
    }
    ChassisVelocities ChassisVelocities = KINEMATICS.toChassisVelocities(observation.moduleStates);
    robotAccelerations =
      new Twist2d(
        (ChassisVelocities.vx - m_lastChassisVelocities.vx) / (observation.timestamp - lastTimestamp),
        (ChassisVelocities.vy - m_lastChassisVelocities.vy) / (observation.timestamp - lastTimestamp),
        (ChassisVelocities.omega - m_lastChassisVelocities.omega) / (observation.timestamp - lastTimestamp)
      );

    m_lastChassisVelocities = ChassisVelocities;
    lastTimestamp = observation.timestamp;
    Logger.recordOutput("CatzRobotTracker/ChassisVelocities", Math.hypot(m_lastChassisVelocities.vx, m_lastChassisVelocities.vy));
    // Calculate diff from last odometry pose and add onto pose estimate

    Logger.recordOutput("CatzRobotTracker/EstimatedPose", estimatedPose);
  } // end of addOdometryObservation

  public Twist2d getRobotAccelerations(){
    return robotAccelerations;
  }

  /**
   * Reset estimated pose and odometry pose to pose <br>
   * Clear pose buffer
   */
  public void resetPose(Pose2d initialPose) {
    // System.out.println(initialPose.getRotation().getDegrees());
    estimatedPose = initialPose;
    odometryPose = initialPose;
  }

  public void resetPose(Supplier<Pose2d> initialPoseSupplier) {
    // System.out.println(initialPose.getRotation().getDegrees());
    estimatedPose = initialPoseSupplier.get();
    odometryPose = initialPoseSupplier.get();
  }

  // ------------------------------------------------------------------------------------------------------
  //
  //  CatzRobotTracker Getters
  //
  // ------------------------------------------------------------------------------------------------------

  public ChassisVelocities getRobotRelativeChassisVelocities() {
    return m_lastChassisVelocities;
  }

  public ChassisVelocities getFieldRelativeChassisVelocities() {
    return m_lastChassisVelocities.toFieldRelative(estimatedPose.getRotation());
  }

  /********************************************************************************************************************************
   *
   *
   ********************************************************************************************************************************/
  public record OdometryObservation(
      SwerveModulePosition[] wheelPositions,
      SwerveModuleVelocity[] moduleStates,
      Rotation2d gyroAngle,
      double timestamp) {}

  public static CatzRobotTracker getInstance() {
    if (Instance == null) {
      Instance = new CatzRobotTracker();
    }
    return Instance;
  }
}
