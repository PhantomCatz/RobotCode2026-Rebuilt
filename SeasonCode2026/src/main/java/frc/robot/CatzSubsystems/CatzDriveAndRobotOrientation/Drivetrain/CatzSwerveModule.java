package frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain;

import static frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants.*;


import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.BaseStatusSignal; // Import added

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CatzConstants;
import frc.robot.Utilities.Alert;
import frc.robot.Utilities.CatzMathUtils;
import frc.robot.Utilities.CatzMathUtils.Conversions;
import frc.robot.Utilities.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class CatzSwerveModule {

  // Module delcaration block
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  // Module Strings for Logging
  private final String m_moduleName;

  // OPTIMIZATION: Pre-calculate logging keys to avoid string concatenation in loops
  private final String logKeyDriveRecFPS;
  private final String logKeyDriveTargFPS;
  private final String logKeyCurModState;
  private final String logKeyAngleErr;
  private final String logKeyCurModAng;
  private final String logKeyAbsEnc;
  private final String smartDashAbsEnc;
  private final String smartDashAngle;
  private final String motorOutputs;

  // Global swerve module variables
  private SwerveModuleState m_swerveModuleState = new SwerveModuleState();

  // Alerts
  private final Alert driveMotorDisconnected;
  private final Alert steerMotorDisconnected;

  public CatzSwerveModule(ModuleIDs config, String moduleName) {
    this.m_moduleName = moduleName;

    // Init Logging Strings once
    logKeyDriveRecFPS = "Module " + m_moduleName + "/drive recorded fps";
    logKeyDriveTargFPS = "Module " + m_moduleName + "/drive target fps";
    logKeyCurModState = "Module " + m_moduleName + "/currentmodule state";
    logKeyAngleErr = "Module " + m_moduleName + "/angle error deg";
    logKeyCurModAng = "Module " + m_moduleName + "/currentmoduleangle rad";
    logKeyAbsEnc = "Module " + m_moduleName + "/current absolute enc";
    smartDashAbsEnc = "absencposrad" + m_moduleName;
    smartDashAngle = "angle" + m_moduleName;
    motorOutputs = "RealInputs/Drive/Motors " + m_moduleName;

    // Run Subsystem disconnect check
    if (DriveConstants.IS_DRIVE_DISABLED) {
      io = new ModuleIONull();
      System.out.println("Module " + m_moduleName + " Unconfigured");
    } else {
      // Run Robot Mode hardware assignment
      switch (CatzConstants.hardwareMode) {
        case REAL:
          io = new ModuleIORealFoc(config, m_moduleName);
          System.out.println("Module " + m_moduleName + " Configured for Real");
          break;
        case REPLAY:
          io = new ModuleIORealFoc(config, m_moduleName) {};
          System.out.println("Module " + m_moduleName + " Configured for Replay simulation");
          break;
        case SIM:
          io = new ModuleIOSim(config);
          System.out.println("Module " + m_moduleName + " Configured for WPILIB simulation");
          break;
        default:
          io = null;
          System.out.println("Module " + m_moduleName + " Unconfigured");
          break;
      }
    }

    // Disconnected Alerts
    driveMotorDisconnected =
        new Alert(m_moduleName + " drive motor disconnected!", Alert.AlertType.kError);
    steerMotorDisconnected =
        new Alert(m_moduleName + " steer motor disconnected!", Alert.AlertType.kError);

    resetDriveEncs();
  }

  // Pass signals up to the subsystem
  public BaseStatusSignal[] getPhoenixSignals() {
      return io.getSignals();
  }

  double prevCur = 0.0;
  public void periodic() {
    // Process and Log Module Inputs
    io.updateInputs(inputs);
    Logger.processInputs(motorOutputs, inputs);

    // Update ff and controllers
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setDrivePID(drivekP.get(), 0, drivekD.get()), drivekP, drivekD);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setSteerPID(steerkP.get(), 0, steerkD.get()), steerkP, steerkD);

    // Display alerts
    driveMotorDisconnected.set(!inputs.isDriveMotorConnected);
    steerMotorDisconnected.set(!inputs.isSteerMotorConnected);



  }

  public void debugLogsSwerve() {
    // OPTIMIZATION: Use pre-cached keys
    Logger.recordOutput(logKeyDriveRecFPS, Units.metersToFeet(Conversions.RPSToMPS(inputs.driveVelocityRPS)));
    Logger.recordOutput(logKeyDriveTargFPS, Units.metersToFeet(m_swerveModuleState.speedMetersPerSecond));
    Logger.recordOutput(logKeyCurModState, m_swerveModuleState.angle.getRadians());
    Logger.recordOutput(logKeyAngleErr, Math.toDegrees(m_swerveModuleState.angle.getRadians() - getAbsEncRadians()));
    Logger.recordOutput(logKeyCurModAng, getAbsEncRadians());
    Logger.recordOutput(logKeyAbsEnc, inputs.rawAbsEncValueRotation);

    SmartDashboard.putNumber(smartDashAngle, getCurrentRotation().getDegrees());
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setModuleAngleAndVelocity(SwerveModuleState state) {
    this.m_swerveModuleState = state;
    double targetAngleRads = state.angle.getRadians();
    double currentAngleRads = getAbsEncRadians();

    io.runDriveVelocityRPSIO(Conversions.MPSToRPS(state.speedMetersPerSecond));
    io.runSteerPositionSetpoint(currentAngleRads, targetAngleRads);
  }

  public void setSteerPower(double pwr) {
    io.runSteerPercentOutput(pwr);
  }

  public void setDriveVelocity(double velocity) {
    io.runDriveVelocityRPSIO(velocity);
  }

  public void stopDriving() {
    io.runDriveVelocityRPSIO(0.0);
  }

  public void setNeutralModeDrive(NeutralModeValue type) {
    io.setDriveNeutralModeIO(type);
  }

  public void setNeutralModeSteer(NeutralModeValue type) {
    io.setSteerNeutralModeIO(type);
  }

  public void resetDriveEncs() {
    io.setDrvSensorPositionIO(0.0);
  }

  public SwerveModuleState optimizeWheelAngles(SwerveModuleState unoptimizedState) {
    SwerveModuleState optimizedState =
        CatzMathUtils.optimize(unoptimizedState, getCurrentRotation());
    return optimizedState;
  }

  public SwerveModuleState getModuleState() {
    double velocityMPS = CatzMathUtils.Conversions.RPSToMPS(inputs.driveVelocityRPS);
    return new SwerveModuleState(velocityMPS, getCurrentRotation());
  }

  public SwerveModuleState getModuleStateSetpoint() {
    return m_swerveModuleState;
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(getDriveDistanceMeters(), getCurrentRotation());
  }

  public double getDriveDistanceMeters() {
    return CatzMathUtils.Conversions.RPSToMPS(inputs.drivePositionUnits);
  }

  public double getPositionRads() {
    return Units.rotationsToRadians(inputs.drivePositionUnits);
  }

  public Rotation2d getAngle() {
    return inputs.steerAbsPosition;
  }

  public double getCharacterizationVelocityRadPerSec() {
    return Units.rotationsToRadians(getDrvVelocityRPS());
  }

  public double getDrvVelocityRPS() {
    return inputs.driveVelocityRPS;
  }

  public Rotation2d getCurrentRotation() {
    return new Rotation2d(getAbsEncRadians());
  }

  private double getAbsEncRadians() {
    return inputs.steerAbsPosition.getRadians();
  }
}
