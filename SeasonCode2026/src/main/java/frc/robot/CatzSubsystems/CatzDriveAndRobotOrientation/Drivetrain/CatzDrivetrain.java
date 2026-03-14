package frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain;

import static frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.signals.NeutralModeValue;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker.OdometryObservation;
import frc.robot.Robot;
import frc.robot.Utilities.Alert;
import frc.robot.Utilities.HolonomicDriveController;
import frc.robot.Utilities.LoggedTunableNumber;
import frc.robot.Utilities.ModuleLimits;
import frc.robot.Utilities.SwerveSetpoint;
import frc.robot.Utilities.SwerveSetpointGenerator;

import java.util.ArrayList;
import java.util.Arrays;
// import org.littletonrobotics.junction.AutoLogOutput;
// import org.littletonrobotics.junction.Logger;
import java.util.Collections;
import java.util.List;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// Drive train subsystem for swerve drive implementation
public class CatzDrivetrain extends SubsystemBase {
  private static CatzDrivetrain Instance;

  // Gyro input/output interface
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  // Alerts
  private final Alert gyroDisconnected;

  // Array of swerve modules representing each wheel in the drive train
  private CatzSwerveModule[] m_swerveModules = new CatzSwerveModule[4];
  private SwerveModuleState[] optimizedDesiredStates = new SwerveModuleState[4];

  // Swerve modules representing each corner of the robot
  public final CatzSwerveModule RT_FRNT_MODULE;
  public final CatzSwerveModule RT_BACK_MODULE;
  public final CatzSwerveModule LT_BACK_MODULE;
  public final CatzSwerveModule LT_FRNT_MODULE;

  private HolonomicDriveController hoController = DriveConstants.getNewHolController();

  private final Field2d field;

  private BaseStatusSignal[] allSignals;

  public double timeToReachTrench = 0.0;

  private SwerveSetpoint currentSetpoint = new SwerveSetpoint(
      new ChassisSpeeds(),
      new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
      });

  private final SwerveSetpointGenerator swerveSetpointGenerator;

  private CatzDrivetrain() {

    // Gyro Instantiation
    switch (CatzConstants.hardwareMode) {
      case REAL:
        gyroIO = new GyroIOPigeon();
        break;
      case REPLAY:
        gyroIO = new GyroIOPigeon() {
        };
        break;
      default:
        gyroIO = null;
        break;
    }
    gyroDisconnected = new Alert("Gyro disconnected!", Alert.AlertType.kWarning);

    // Create swerve modules for each corner of the robot
    RT_FRNT_MODULE = new CatzSwerveModule(DriveConstants.MODULE_CONFIGS[INDEX_FR], MODULE_NAMES[INDEX_FR]);
    RT_BACK_MODULE = new CatzSwerveModule(DriveConstants.MODULE_CONFIGS[INDEX_BR], MODULE_NAMES[INDEX_BR]);
    LT_BACK_MODULE = new CatzSwerveModule(DriveConstants.MODULE_CONFIGS[INDEX_BL], MODULE_NAMES[INDEX_BL]);
    LT_FRNT_MODULE = new CatzSwerveModule(DriveConstants.MODULE_CONFIGS[INDEX_FL], MODULE_NAMES[INDEX_FL]);

    // Assign swerve modules to the array for easier access
    m_swerveModules[INDEX_FR] = RT_FRNT_MODULE;
    m_swerveModules[INDEX_BR] = RT_BACK_MODULE;
    m_swerveModules[INDEX_BL] = LT_BACK_MODULE;
    m_swerveModules[INDEX_FL] = LT_FRNT_MODULE;

    if (CatzConstants.hardwareMode == CatzConstants.RobotHardwareMode.REAL ||
        CatzConstants.hardwareMode == CatzConstants.RobotHardwareMode.REPLAY) {
      List<BaseStatusSignal> signalList = new ArrayList<>();
      for (CatzSwerveModule module : m_swerveModules) {
        Collections.addAll(signalList, module.getPhoenixSignals());
      }
      allSignals = signalList.toArray(new BaseStatusSignal[0]);
    } else {
      allSignals = new BaseStatusSignal[0];
    }

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    swerveSetpointGenerator = new SwerveSetpointGenerator(SWERVE_KINEMATICS, MODULE_TRANSLATIONS);
  }

  @Override
  public void periodic() {
    // ----------------------------------------------------------------------------------------------------
    // Update inputs (sensors/encoders) for code logic and advantage kit
    // ----------------------------------------------------------------------------------------------------
    if (allSignals.length > 0) {
      BaseStatusSignal.refreshAll(allSignals);
    }

    for (CatzSwerveModule module : m_swerveModules) {
      module.periodic();
    }

    // -----------------------------------------------------------------------------------------------------
    // Attempt to update gyro inputs and log
    // -----------------------------------------------------------------------------------------------------
    try {
      gyroIO.updateInputs(gyroInputs);
    } catch (Exception e) {

    }
    // NOTE Gyro needs to be firmly mounted to rio for accurate results.
    // Set Gyro Disconnect alert to go off when gyro is disconnected
    if (Robot.isReal()) {
      gyroDisconnected.set(!gyroInputs.gyroConnected);
    }

    // ----------------------------------------------------------------------------------------------------
    // Swerve drive Odometry and Velocity updates
    // ----------------------------------------------------------------------------------------------------
    SwerveModulePosition[] wheelPositions = getModulePositions();
    // Grab latest gyro measurments
    Rotation2d gyroAngle2d = (CatzConstants.hardwareMode == CatzConstants.RobotHardwareMode.SIM)
        ? null
        : getRotation2d();

    // Add observations to robot tracker
    OdometryObservation observation = new OdometryObservation(
        wheelPositions,
        getModuleStates(),
        gyroAngle2d,
        Timer.getFPGATimestamp());
    CatzRobotTracker.Instance.addOdometryObservation(observation);
  } // end of drivetrain periodic

  // --------------------------------------------------------------------------------------------------------------------------
  //
  // Driving methods
  //
  // --------------------------------------------------------------------------------------------------------------------------
  public ChassisSpeeds appliedChassisSpeeds = new ChassisSpeeds();

  public void drive(ChassisSpeeds chassisSpeeds) {
    appliedChassisSpeeds = chassisSpeeds;
    ChassisSpeeds descreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, CatzConstants.LOOP_TIME);
    // --------------------------------------------------------
    // Convert chassis speeds to individual module states and set module states
    // --------------------------------------------------------
    SwerveModuleState[] unoptimizedModuleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(descreteSpeeds);
    // --------------------------------------------------------
    // Scale down wheel speeds
    // --------------------------------------------------------
    SwerveDriveKinematics.desaturateWheelSpeeds(unoptimizedModuleStates,
        DriveConstants.DRIVE_CONFIG.maxLinearVelocity());
    // --------------------------------------------------------
    // Optimize Wheel Angles
    // --------------------------------------------------------
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state that prevents it from overturn, useful
      // for logging
      optimizedDesiredStates[i] = m_swerveModules[i].optimizeWheelAngles(unoptimizedModuleStates[i]);

      // Set module states to each of the swerve modules
      m_swerveModules[i].setModuleAngleAndVelocity(optimizedDesiredStates[i]);
    }
  }

  public void simpleDrive(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);

    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state that prevents it from overturn, useful
      // for logging
      optimizedDesiredStates[i] = m_swerveModules[i].optimizeWheelAngles(moduleStates[i]);

      // Set module states to each of the swerve modules
      m_swerveModules[i].setModuleAngleAndVelocity(optimizedDesiredStates[i]);
    }
  }

  /**
   * Drives the robot with dynamic acceleration limits to prevent wheel slip.
   * * This method calculates a custom acceleration limit based on:
   * 1. The severity of the direction change (using Dot Product).
   * 2. The current speed of the robot (ignoring limits at low speeds).
   * * @param desiredSpeeds The target chassis speeds from the driver/auto.
   */
  public void slipControlDrive(ChassisSpeeds desiredSpeeds) {
    // The maximum acceleration allowed during a high-speed
    // J-turn or 180-degree reversal. If the robot drifts, lower this.
    double accelSlip = 3.5;

    // The maximum acceleration allowed for straight-line
    // driving, braking, or launching from a stop.
    double accelTraction = 22.0;

    // Below this speed (m/s), we ignore the slip limit
    // because the tires have plenty of grip.
    double safeSpeed = 2.0;

    // Get the robot's current velocity vector from the setpoint generator
    ChassisSpeeds currentSpeeds = CatzRobotTracker.Instance.getRobotRelativeChassisSpeeds();

    // Calculate the magnitude (total speed) of current and desired vectors
    double currentSpeedMag = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
    double targetSpeedMag = Math.hypot(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond);

    double directionalAccelLimit;

    // If the robot is stopped or stopping, direction doesn't matter.
    // Allow maximum traction for launching or braking.
    if (currentSpeedMag < 0.1 || targetSpeedMag < 0.1) {
      directionalAccelLimit = accelTraction;
    } else {
      // Calculate the magnitude of dot product of the velocity vectors:
      // (vx1*vx2 + vy1*vy2) / (|v1| * |v2|)
      // Result ranges from:
      // 1.0 (Aligned) -> Moving in the same direction
      // 0.0 (90 deg) -> Turning sharp corner
      // -1.0 (Opposite) -> Reversing direction
      double dot = (currentSpeeds.vxMetersPerSecond * desiredSpeeds.vxMetersPerSecond
          + currentSpeeds.vyMetersPerSecond * desiredSpeeds.vyMetersPerSecond)
          / (currentSpeedMag * targetSpeedMag);

      // Clamp for floating point errors
      dot = Math.max(-1.0, Math.min(1.0, dot));

      // Map [-1 to 1] -> [0 to 1]
      // 0.0 = We are reversing/turning (Needs Caution)
      // 1.0 = We are driving straight (Needs Speed)
      double directionFactor = (dot + 1.0) / 2.0;

      // linearly interpolate between the Slip Floor and Traction Ceiling
      directionalAccelLimit = accelSlip + (directionFactor * (accelTraction - accelSlip));
    }

    // Calculate how risky our current speed is.
    // 0.0 = Low Speed (Safe) -> We can ignore the directional limit.
    // 1.0 = High Speed (Risky) -> We must strictly obey the directional limit.
    double speedRiskFactor = Math.min(currentSpeedMag / safeSpeed, 1.0);

    // Blend the limits:
    // Low Speed -> Uses kAccelTraction (Max Performance)
    // High Speed -> Uses directionalAccelLimit (Safety)
    double finalAccelLimit = accelTraction + (speedRiskFactor * (directionalAccelLimit - accelTraction));

    // Create the dynamic constraints for this specific loop cycle
    ModuleLimits dynamicLimits = new ModuleLimits(
        DriveConstants.DRIVE_CONFIG.maxLinearVelocity(),
        finalAccelLimit, // <--- The calculated Anti-Slip Acceleration
        DriveConstants.DRIVE_CONFIG.maxAngularVelocity());

    // Discretize desired speeds to correct for robot motion during the loop time
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(desiredSpeeds, 0.02);

    currentSetpoint = swerveSetpointGenerator.generateSetpoint(
        dynamicLimits,
        currentSetpoint,
        discreteSpeeds,
        0.02);

    SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();

    // Log the calculated limits to TunableNumbers/Dashboard for debugging
    Logger.recordOutput("Drive/AccelLimit", finalAccelLimit);
    Logger.recordOutput("Drive/SpeedRiskFactor", speedRiskFactor);

    for (int i = 0; i < 4; i++) {
      // Use your team's custom optimization logic (keeps wheels from spinning >90
      // deg)
      // m_swerveModules[i].optimizeWheelAngles() is assumed to return the optimized
      // state
      SwerveModuleState optimizedState = m_swerveModules[i].optimizeWheelAngles(setpointStates[i]);

      // Ensure we don't apply full power until the wheel is actually pointing
      // in the correct direction. This prevents "skittering" sideways.
      Rotation2d currentAngle = m_swerveModules[i].getAngle();

      // Dot product of Wheel Heading vs Target Heading
      double cosineScale = optimizedState.angle.minus(currentAngle).getCos();

      // Only enforce this if we are moving fast enough to matter (Risk > 10%)
      if (speedRiskFactor > 0.1) {
        // If cosine is negative (error > 90 deg), output is 0.
        optimizedState.speedMetersPerSecond *= Math.max(0.0, cosineScale);
      }
      m_swerveModules[i].setModuleAngleAndVelocity(optimizedState);

      optimizedDesiredStates[i] = optimizedState;
    }
  }


  public void moveWhileShootAccControl(ChassisSpeeds desiredSpeeds) {

    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(desiredSpeeds, 0.02);
    ModuleLimits limits;

    if(CatzSuperstructure.Instance.getIsScoring()){
      limits = DriveConstants.MOVE_WHILE_SHOOT_LIMITS;
    }else{
      limits = DriveConstants.DRIVE_LIMITS;
    }
    currentSetpoint = swerveSetpointGenerator.generateSetpoint(
        limits,
        currentSetpoint,
        discreteSpeeds,
        0.02);

    SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();

    for (int i = 0; i < 4; i++) {
      SwerveModuleState optimizedState = m_swerveModules[i].optimizeWheelAngles(setpointStates[i]);

      m_swerveModules[i].setModuleAngleAndVelocity(optimizedState);

      optimizedDesiredStates[i] = optimizedState;
    }
  }

  /** Create a command to stop driving */
  public void stopDriving() {
    for (CatzSwerveModule module : m_swerveModules) {
      module.stopDriving();
      module.setSteerPower(0.0);
    }
  }

  /** Runs in a circle at omega. */
  public void runWheelRadiusCharacterization(double omegaSpeed) {
    simpleDrive(new ChassisSpeeds(0.0, 0.0, omegaSpeed));
  }

  /** Disables the characterization mode. */
  public void endCharacterization() {
    stopDriving();
  }

  /** Runs forwards at the commanded voltage or amps. */
  public void runCharacterization(double input) {
    simpleDrive(new ChassisSpeeds(0.0, 0.0, input));
  }

  // -----------------------------------------------------------------------------------------------------------
  //
  // Drivetrain Misc Methods
  //
  // -----------------------------------------------------------------------------------------------------------
  /** Get the position of all drive wheels in radians. */
  public double[] getWheelRadiusCharacterizationPosition() {
    return Arrays.stream(m_swerveModules).mapToDouble(CatzSwerveModule::getPositionRads).toArray();
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : m_swerveModules) {
      driveVelocityAverage += module.getCharacterizationVelocityRadPerSec();
    }
    return driveVelocityAverage / 4.0;
  }

  /** Set Neutral mode for all swerve modules */
  public void setDriveNeutralMode(NeutralModeValue type) {
    for (CatzSwerveModule module : m_swerveModules) {
      module.setNeutralModeDrive(type);
    }
  }

  /** Set coast mode for all swerve modules */
  public void setSteerNeutralMode(NeutralModeValue type) {
    for (CatzSwerveModule module : m_swerveModules) {
      module.setNeutralModeSteer(type);
    }
  }

  /** command to cancel running auto trajectories */
  public Command cancelTrajectory() {
    Command cancel = new InstantCommand();
    cancel.addRequirements(this);
    return cancel;
  }

  public void resetDriveEncs() {
    for (CatzSwerveModule module : m_swerveModules) {
      module.resetDriveEncs();
    }
  }

  /**
   * Make sure to run this before every new trajectory
   */
  public void followChoreoTrajectoryInit(AutoTrajectory traj) {
    hoController = DriveConstants.getNewHolController();
  }

  LoggedTunableNumber aff = new LoggedTunableNumber("aff", 2.5);

  /**
   * This function only runs the "execute" portion of a command. Initialization
   * and ending should be done elsewhere.
   *
   * @param sample
   */
  public void followChoreoTrajectoryExecute(SwerveSample sample) {
    // 1. Calculate the denominator (velocity magnitude cubed)
    double velocitySq = (sample.vx * sample.vx) + (sample.vy * sample.vy);
    double velocityMag = Math.sqrt(velocitySq);

    double curvature = 0.0;

    // 2. Protect against division by zero if the robot is stopped
    if (velocityMag > 1e-6) {
      curvature = Math.abs(sample.vx * sample.ay - sample.vy * sample.ax) / (velocitySq * velocityMag);
    }

    Trajectory.State state = new Trajectory.State(
        sample.t,
        velocityMag,
        Math.hypot(sample.ax, sample.ay), // Use raw acceleration here
        new Pose2d(
            new Translation2d(sample.x, sample.y),
            Rotation2d.fromRadians(Math.atan2(sample.vy, sample.vx))),
        curvature // Input the calculated curvature here
    );

    Pose2d curPose = CatzRobotTracker.getInstance().getEstimatedPose();
    ChassisSpeeds adjustedSpeeds = hoController.calculate(curPose, state, Rotation2d.fromRadians(sample.heading));

    Logger.recordOutput("Target Auton Pose", new Pose2d(sample.x, sample.y, Rotation2d.fromRadians(sample.heading)));
    drive(adjustedSpeeds);
  }

  // -----------------------------------------------------------------------------------------------------------
  //
  // Drivetrain Getters
  //
  // -----------------------------------------------------------------------------------------------------------
  /**
   * Dependant on the installation of the gyro, the value of this method may be
   * negative
   *
   * @return The Heading of the robot dependant on where it's been instantiated
   */
  public double getGyroHeading() {
    return gyroInputs.gyroAngle; // Negative on Forte due to instalation, gyro's left is not robot left
  }

  public double getGyroVel() {
    return gyroInputs.gyroYawVel;
  }

  /** Get the Rotation2d object based on the gyro angle */
  private Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getGyroHeading());
  }

  /** Get an array of swerve module states */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    for (int i = 0; i < m_swerveModules.length; i++) {
      moduleStates[i] = m_swerveModules[i].getModuleState();
    }
    return moduleStates;
  }

  /**
   * Returns the measured speeds of the robot in the robot's frame of reference.
   */
  @AutoLogOutput(key = "Drive/MeasuredSpeeds")
  private Twist2d getTwist2dSpeeds() {
    return DriveConstants.SWERVE_KINEMATICS.toTwist2d(getModulePositions());
  }

  /** Get an array of swerve module positions */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    for (int i = 0; i < m_swerveModules.length; i++) {
      modulePositions[i] = m_swerveModules[i].getModulePosition();
    }
    return modulePositions;
  }

  /** Map Circle orientation for wheel radius characterization */
  public static Rotation2d[] getCircleOrientations() {
    return Arrays.stream(DriveConstants.MODULE_TRANSLATIONS)
        .map(translation -> translation.getAngle().plus(new Rotation2d(Math.PI / 2.0)))
        .toArray(Rotation2d[]::new);
  }

  public static CatzDrivetrain getInstance() {
    if (Instance == null) {
      Instance = new CatzDrivetrain();
    }
    return Instance;
  }

}
