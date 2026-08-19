package frc.robot.Commands.DriveAndRobotOrientationCmds;

import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.driverstation.Alliance;
import org.wpilib.command2.Command;
import frc.robot.CatzConstants.XboxInterfaceConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

import frc.robot.Utilities.SwerveSetpoint;
import frc.robot.Utilities.SwerveSetpointGenerator;

/**************************************************************************************************
 *
 * TeleopDriveCmd
 *
 **************************************************************************************************/

public class TeleopDriveCmd extends Command {
  // Subsystem declaration
  private final CatzDrivetrain m_drivetrain;

  // Xbox controller buttons
  private final Supplier<Double> m_headingPctOutput_X;
  private final Supplier<Double> m_headingPctOutput_Y;
  private final Supplier<Double> m_angVelocityPctOutput;

  // drive variables
  private double joyX;
  private double joyY;
  private double turningVelocity;

  private ChassisVelocities ChassisVelocities;
  private SwerveSetpoint currentSetpoint = new SwerveSetpoint(
      new ChassisVelocities(),
      new SwerveModuleVelocity[] {
          new SwerveModuleVelocity(),
          new SwerveModuleVelocity(),
          new SwerveModuleVelocity(),
          new SwerveModuleVelocity()
      });
  private final SwerveSetpointGenerator swerveSetpointGenerator;

  // --------------------------------------------------------------------------------------
  //
  // Teleop Drive Command Constructor
  //
  // --------------------------------------------------------------------------------------
  public TeleopDriveCmd(
      Supplier<Double> supplierLeftJoyX,
      Supplier<Double> supplierLeftJoyY,
      Supplier<Double> angVelocityPctOutput,
      CatzDrivetrain drivetrain) {
    // Chassis magnatude and direction control
    this.m_headingPctOutput_X = supplierLeftJoyX;
    this.m_headingPctOutput_Y = supplierLeftJoyY;
    this.m_angVelocityPctOutput = angVelocityPctOutput;

    // subsystem assignment
    this.m_drivetrain = drivetrain;
    System.out.println("TeleopDriveCmd drivetrain = " + drivetrain);

    addRequirements(this.m_drivetrain);

    swerveSetpointGenerator = new SwerveSetpointGenerator(DriveConstants.SWERVE_KINEMATICS, DriveConstants.MODULE_TRANSLATIONS);
  }

  // --------------------------------------------------------------------------------------
  //
  // Initialize
  //
  // --------------------------------------------------------------------------------------
  @Override
  public void initialize() {
  }

  // --------------------------------------------------------------------------------------
  //
  // Execute
  //
  // --------------------------------------------------------------------------------------
  @Override
  public void execute() {
    // Obtain realtime joystick inputs with supplier methods
    joyX = -m_headingPctOutput_Y.get(); // Raw accel
    joyY = -m_headingPctOutput_X.get();
    turningVelocity = -m_angVelocityPctOutput.get(); // alliance flip shouldn't change for turing speed when switching
                                                     // alliances

    // Flip Directions for left joystick if alliance is red
    if (DriverStationBackend.getAlliance().orElse(Alliance.BLUE) == Alliance.RED) {
      joyX = -joyX;
      joyY = -joyY;
    }

    double currentMagnitude = Math.hypot(joyX, joyY);

    double finalVelX = 0.0;
    double finalVelY = 0.0;

    // Normal teleop driving logic
    if (currentMagnitude > XboxInterfaceConstants.kDeadband) {
      finalVelX = joyX * DriveConstants.DRIVE_CONFIG.maxLinearVelocity();
      finalVelY = joyY * DriveConstants.DRIVE_CONFIG.maxLinearVelocity();
    }

    turningVelocity = Math.abs(turningVelocity) > XboxInterfaceConstants.kDeadband
        ? turningVelocity * DriveConstants.DRIVE_CONFIG.maxAngularVelocity()
        : 0.0;

   // Construct desired chassis speeds normally
    ChassisVelocities = ChassisVelocities.fromFieldRelativeSpeeds(finalVelX,
        finalVelY,
        turningVelocity,
        CatzRobotTracker.getInstance().getEstimatedPose().getRotation());

    // Artificially cap the target translation speed if scoring
    if(CatzSuperstructure.Instance.getIsScoring()) {
      double maxScoringVel = DriveConstants.MOVE_WHILE_SHOOT_LIMITS.maxDriveVelocity();
      double currentTargetVel = Math.hypot(
          ChassisVelocities.vx,
          ChassisVelocities.vy
      );

      // Scale down linear translation if it exceeds the scoring speed limit
      if (currentTargetVel > maxScoringVel) {
        double scale = maxScoringVel / currentTargetVel;
        ChassisVelocities.vx *= scale;
        ChassisVelocities.vy *= scale;
      }
    }

    // ALWAYS use DRIVE_LIMITS so the robot can brake rotation instantly
    currentSetpoint = swerveSetpointGenerator.generateSetpoint(
      DriveConstants.DRIVE_LIMITS,
      currentSetpoint,
      ChassisVelocities,
      0.02);

    // Send new ChassisVelocities object to the drivetrain queue to use later
    CatzDrivetrain.getInstance().swerveSetpointDrive(currentSetpoint);
    // Logger.recordOutput("cur controller input", Math.hypot(currentSetpoint.ChassisVelocities().vx, currentSetpoint.ChassisVelocities().vy));
    // debugLogsDrive();
  } // end of execute()

  // --------------------------------------------------------------------------------------
  //
  // Debug Logging
  //
  // --------------------------------------------------------------------------------------
  public void debugLogsDrive() {
    Logger.recordOutput("Drive/robot orientation rad per sec", ChassisVelocities.omega);
    Logger.recordOutput("Drive/chassisspeed x speed mtr sec", ChassisVelocities.vx);
    Logger.recordOutput("Drive/chassisspeed y speed mtr sec", ChassisVelocities.vy);
  }

  // --------------------------------------------------------------------------------------
  //
  // End
  //
  // --------------------------------------------------------------------------------------
  @Override
  public void end(boolean interrupted) {
  }

  // --------------------------------------------------------------------------------------
  //
  // Is Finished
  //
  // --------------------------------------------------------------------------------------
  @Override
  public boolean isFinished() {
    return false;
  }
}
