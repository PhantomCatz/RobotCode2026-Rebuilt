package frc.robot.Commands.DriveAndRobotOrientationCmds;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
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

  private ChassisSpeeds chassisSpeeds;
  private SwerveSetpoint currentSetpoint = new SwerveSetpoint(
      new ChassisSpeeds(),
      new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
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
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
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
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(finalVelX,
        finalVelY,
        turningVelocity,
        CatzRobotTracker.getInstance().getEstimatedPose().getRotation());

    // Artificially cap the target translation speed if scoring
    if(CatzSuperstructure.Instance.getIsScoring()) {
      double maxScoringVel = DriveConstants.MOVE_WHILE_SHOOT_LIMITS.maxDriveVelocity();
      double currentTargetVel = Math.hypot(
          chassisSpeeds.vxMetersPerSecond,
          chassisSpeeds.vyMetersPerSecond
      );

      // Scale down linear translation if it exceeds the scoring speed limit
      if (currentTargetVel > maxScoringVel) {
        double scale = maxScoringVel / currentTargetVel;
        chassisSpeeds.vxMetersPerSecond *= scale;
        chassisSpeeds.vyMetersPerSecond *= scale;
      }
    }

    // ALWAYS use DRIVE_LIMITS so the robot can brake rotation instantly
    currentSetpoint = swerveSetpointGenerator.generateSetpoint(
      DriveConstants.DRIVE_LIMITS,
      currentSetpoint,
      chassisSpeeds,
      0.02);

    // Send new chassisspeeds object to the drivetrain queue to use later
    CatzDrivetrain.getInstance().swerveSetpointDrive(currentSetpoint);
    // Logger.recordOutput("cur controller input", Math.hypot(currentSetpoint.chassisSpeeds().vxMetersPerSecond, currentSetpoint.chassisSpeeds().vyMetersPerSecond));
    // debugLogsDrive();
  } // end of execute()

  // --------------------------------------------------------------------------------------
  //
  // Debug Logging
  //
  // --------------------------------------------------------------------------------------
  public void debugLogsDrive() {
    Logger.recordOutput("Drive/robot orientation rad per sec", chassisSpeeds.omegaRadiansPerSecond);
    Logger.recordOutput("Drive/chassisspeed x speed mtr sec", chassisSpeeds.vxMetersPerSecond);
    Logger.recordOutput("Drive/chassisspeed y speed mtr sec", chassisSpeeds.vyMetersPerSecond);
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
