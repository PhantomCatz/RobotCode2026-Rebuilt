package frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain;

import static frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.system.Timer;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker.OdometryObservation;
import frc.robot.Robot;
import frc.robot.Utilities.Alert;
import frc.robot.Utilities.SwerveSetpoint;

import java.util.ArrayList;
import java.util.Arrays;
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
  private SwerveModuleVelocity[] optimizedDesiredStates = new SwerveModuleVelocity[4];

  // Swerve modules representing each corner of the robot
  public final CatzSwerveModule RT_FRNT_MODULE;
  public final CatzSwerveModule RT_BACK_MODULE;
  public final CatzSwerveModule LT_BACK_MODULE;
  public final CatzSwerveModule LT_FRNT_MODULE;

  private final Field2d field;

  private BaseStatusSignal[] allSignals;

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
        Timer.getTimestamp());
    CatzRobotTracker.getInstance().addOdometryObservation(observation);

  } // end of drivetrain periodic

  // --------------------------------------------------------------------------------------------------------------------------
  //
  // Driving methods
  //
  // --------------------------------------------------------------------------------------------------------------------------
  public void drive(ChassisVelocities ChassisVelocities) {
    ChassisVelocities descreteSpeeds = ChassisVelocities.discretize(CatzConstants.LOOP_TIME);
    // --------------------------------------------------------
    // Convert chassis speeds to individual module states and set module states
    // --------------------------------------------------------
    SwerveModuleVelocity[] unoptimizedModuleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleVelocities(descreteSpeeds);
    // --------------------------------------------------------
    // Scale down wheel speeds
    // --------------------------------------------------------
    unoptimizedModuleStates = SwerveDriveKinematics.desaturateWheelVelocities(unoptimizedModuleStates,
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

  public void simpleDrive(ChassisVelocities speeds) {
    SwerveModuleVelocity[] moduleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleVelocities(speeds);

    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state that prevents it from overturn, useful
      // for logging
      optimizedDesiredStates[i] = m_swerveModules[i].optimizeWheelAngles(moduleStates[i]);

      // Set module states to each of the swerve modules
      m_swerveModules[i].setModuleAngleAndVelocity(optimizedDesiredStates[i]);
    }
  }

  public void swerveSetpointDrive(SwerveSetpoint setpoint) {
    SwerveModuleVelocity[] setpointStates = setpoint.moduleStates();

    for (int i = 0; i < 4; i++) {
      SwerveModuleVelocity optimizedState = m_swerveModules[i].optimizeWheelAngles(setpointStates[i]);

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
    simpleDrive(new ChassisVelocities(0.0, 0.0, omegaSpeed));
  }

  /** Disables the characterization mode. */
  public void endCharacterization() {
    stopDriving();
  }

  /** Runs forwards at the commanded voltage or amps. */
  public void runCharacterization(double input) {
    simpleDrive(new ChassisVelocities(0.0, 0.0, input));
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

  /** Set current limits for normal driving*/
  public void setNormalConfig() {
    for (CatzSwerveModule module : m_swerveModules) {
      module.setNormalConfig();
    }
    Logger.recordOutput("Drive Config", "normal");
  }

  public void resetDriveEncs() {
    for (CatzSwerveModule module : m_swerveModules) {
      module.resetDriveEncs();
    }
  }

  public void setXLock() {
      for (int i = 0; i < 4; i++) {
          m_swerveModules[i].setModuleAngleAndVelocity(DriveConstants.xLockStates[i]);
      }
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
  public SwerveModuleVelocity[] getModuleStates() {
    SwerveModuleVelocity[] moduleStates = new SwerveModuleVelocity[4];
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
