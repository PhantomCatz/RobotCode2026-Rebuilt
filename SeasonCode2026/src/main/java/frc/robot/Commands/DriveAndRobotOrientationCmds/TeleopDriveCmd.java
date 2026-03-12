package frc.robot.Commands.DriveAndRobotOrientationCmds;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

  private double lockedDriveDirectionX = 0.0;
  private double lockedDriveDirectionY = 0.0;
  private boolean wasSpeeding = false;

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
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      joyX = -joyX;
      joyY = -joyY;
    }

    boolean isScoring = CatzSuperstructure.Instance.getIsScoring();
    boolean isSpeeding = CatzSuperstructure.Instance.getIsSpeeding();
    double currentMagnitude = Math.hypot(joyX, joyY);

    double finalVelX = 0.0;
    double finalVelY = 0.0;

    turningVelocity = Math.abs(turningVelocity) > XboxInterfaceConstants.kDeadband
        ? turningVelocity * DriveConstants.DRIVE_CONFIG.maxAngularVelocity()
        : 0.0;

    if (isScoring) {
      if (isSpeeding) {
        System.out.println("speeding");
        if (!wasSpeeding) {
          System.out.println("start locking");
          // Button was just pressed
          // Lock in the exact speed and direction you are currently driving
          if (currentMagnitude > XboxInterfaceConstants.kDeadband) {
            // Drive at max speed in the joystick direction
            double joySpeed = Math.hypot(joyX, joyY);
            double scale = DriveConstants.DRIVE_CONFIG.maxLinearVelocity() / joySpeed;
            lockedDriveDirectionX = joyX * scale;
            lockedDriveDirectionY = joyY * scale;
          } else {
            // Don't move
            lockedDriveDirectionX = 0.0;
            lockedDriveDirectionY = 0.0;
          }
        }
        // Drive with the locked speeds
        CatzDrivetrain.getInstance().speedingAccControl(new ChassisSpeeds(lockedDriveDirectionX, lockedDriveDirectionY, turningVelocity));
      }
      else {
        System.out.println("drive slow");
        // Scoring but not speeding, so drive slow
        finalVelX = joyX * DriveConstants.SHOOT_WHILE_MOVE_MOVE_SCALAR * DriveConstants.DRIVE_CONFIG.maxLinearVelocity();
        finalVelY = joyY* DriveConstants.SHOOT_WHILE_MOVE_MOVE_SCALAR * DriveConstants.DRIVE_CONFIG.maxLinearVelocity();
      }
    }
    else {
      System.out.println("drive normal");
      // Normal teleop driving logic
      if (currentMagnitude > XboxInterfaceConstants.kDeadband) {
        finalVelX = joyX * DriveConstants.DRIVE_CONFIG.maxLinearVelocity();
        finalVelY = joyY * DriveConstants.DRIVE_CONFIG.maxLinearVelocity();
      }

      // Reset locks
      lockedDriveDirectionX = 0.0;
      lockedDriveDirectionY = 0.0;
    }
    wasSpeeding = isSpeeding;

    // Construct desired chassis speeds
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(finalVelX,
        finalVelY,
        turningVelocity,
        CatzRobotTracker.getInstance().getEstimatedPose().getRotation());
    // Send new chassisspeeds object to the drivetrain. Don't send if speeding because you already sent earlier
    if (!isSpeeding) {
      m_drivetrain.drive(chassisSpeeds);
    }
    debugLogsDrive();
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
