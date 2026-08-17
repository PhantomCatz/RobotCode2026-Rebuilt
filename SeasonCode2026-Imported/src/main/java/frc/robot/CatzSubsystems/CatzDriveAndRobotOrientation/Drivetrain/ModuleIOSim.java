package frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain;


import org.wpilib.math.controller.PIDController;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.system.LinearSystem;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.LinearSystemUtil;
import org.wpilib.math.util.Units;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.simulation.DCMotorSim;
import frc.robot.CatzConstants;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants.ModuleIDs;

public class ModuleIOSim implements ModuleIO {
  private final LinearSystem<N2, N1, N2> plantDrive =
      LinearSystemUtil.createDCMotorSystem(
          DCMotor.getKrakenX60(1), 0.025, DriveConstants.MODULE_GAINS_AND_RATIOS.driveReduction());
  private final LinearSystem<N2, N1, N2> plantSteer =
      LinearSystemUtil.createDCMotorSystem(
          DCMotor.getKrakenX60(1), 0.004, DriveConstants.MODULE_GAINS_AND_RATIOS.driveReduction());

  private final DCMotorSim driveSim =
      new DCMotorSim(plantDrive, DCMotor.getKrakenX60Foc(1), 0.0, 0.0);
  private final DCMotorSim steerSim =
      new DCMotorSim(plantSteer, DCMotor.getKrakenX60Foc(1), 0.0, 0.0);

  private final PIDController driveFeedback =
      new PIDController(0.1, 0.0, 0.0, CatzConstants.LOOP_TIME);
  private final PIDController steerFeedback =
      new PIDController(10.0, 0.0, 0.0, CatzConstants.LOOP_TIME);

  private double driveAppliedVolts = 0.0;
  private double steerAppliedVolts = 0.0;
  private final Rotation2d steerAbsoluteInitPosition;

  private boolean driveCoast = false;
  private SlewRateLimiter driveVoltsLimiter = new SlewRateLimiter(2.5);

  public ModuleIOSim(ModuleIDs config) {
    steerAbsoluteInitPosition =
        Rotation2d.fromRadians(Units.rotationsToRadians(config.absoluteEncoderOffset()));
    steerFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    if (driveCoast && DriverStationBackend.isDisabled()) {
      runDriveVolts(driveVoltsLimiter.calculate(driveAppliedVolts));
    } else {
      driveVoltsLimiter.reset(driveAppliedVolts);
    }

    driveSim.update(CatzConstants.LOOP_TIME);
    steerSim.update(CatzConstants.LOOP_TIME);

    inputs.driveVelocityRPS = driveSim.getAngularVelocity();
    inputs.drivePositionUnits =
        driveSim.getAngularPosition() / (2 * Math.PI) * 4; // Fudged number to get better result
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveSupplyCurrentAmps = Math.abs(driveSim.getCurrentDraw());

    inputs.steerAbsPosition =
        new Rotation2d(steerSim.getAngularPosition()).plus(steerAbsoluteInitPosition);
    inputs.steerPosition = Rotation2d.fromRadians(steerSim.getAngularPosition());
    inputs.steerVelocityRadsPerSec = steerSim.getAngularVelocity();
    inputs.steerSupplyCurrentAmps = steerAppliedVolts;
    inputs.steerSupplyCurrentAmps = Math.abs(steerSim.getCurrentDraw());
  }

  private void runDriveVolts(double volts) {
    driveAppliedVolts = Math.clamp(volts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  private void runsteerVolts(double volts) {
    steerAppliedVolts = Math.clamp(volts, -12.0, 12.0);
    steerSim.setInputVoltage(steerAppliedVolts);
  }

  @Override
  public void runCharacterization(double input) {
    runDriveVolts(input);
  }

  @Override
  public void runDriveVelocityRPSIO(double velocityRPS) {
    double velocityRadsPerSec = Units.rotationsToRadians(velocityRPS);
    // runDriveVolts has no internal PID so autonomous paths will drift after ending
    // (driveFeedback won't immediately return 0 - it needs to be continuously called to converge to 0 like any other PID,
    //  but trajectoryDriveCommand end() only calls stopDriving once)
    // TLDR ignore autonomous paths drifting in sim because it won't happen in real life since setControl has an internal PID running in a separate thread

    // driveSim.setAngularVelocity(velocityRadsPerSec);

    runDriveVolts(
        driveFeedback.calculate(driveSim.getAngularVelocity(), velocityRadsPerSec));
  }

  @Override
  public void runSteerPositionSetpoint(double currentAngleRad, double angleRads) {
    runsteerVolts(steerFeedback.calculate(steerSim.getAngularPosition(), angleRads));
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveFeedback.setPID(kP, kI, kD);
  }

  @Override
  public void setSteerPID(double kP, double kI, double kD) {
    steerFeedback.setPID(kP, kI, kD);
  }
}
