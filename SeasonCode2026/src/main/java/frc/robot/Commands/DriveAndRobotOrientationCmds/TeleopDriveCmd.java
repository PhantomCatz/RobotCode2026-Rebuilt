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
  private double lockedSpeed = 0.0;
  private boolean wasScoring = false;
  private static final double SHOOTING_JOYSTICK_DEADBAND = 0.4;

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
    double currentMagnitude = Math.hypot(joyX, joyY);

    double finalVelX = 0.0;
    double finalVelY = 0.0;

    if (isScoring) {
      if (!wasScoring) {
        // Button was just pressed
        // Lock in the exact speed and direction you are currently driving
        if (currentMagnitude > XboxInterfaceConstants.kDeadband) {
          lockedDriveDirectionX = joyX * DriveConstants.DRIVE_CONFIG.maxLinearVelocity();
          lockedDriveDirectionY = joyY * DriveConstants.DRIVE_CONFIG.maxLinearVelocity();

          // Calculate and save the absolute speed
          lockedSpeed = Math.hypot(lockedDriveDirectionX, lockedDriveDirectionY);
        } else {
          lockedDriveDirectionX = 0.0;
          lockedDriveDirectionY = 0.0;
          lockedSpeed = 0.0;
        }
      } else if (currentMagnitude > SHOOTING_JOYSTICK_DEADBAND) {
        // Button is being held and driver pushed stick hard to change direction
        // Normalize the joystick vector and multiply by our saved lockedSpeed
        lockedDriveDirectionX = (joyX / currentMagnitude) * lockedSpeed;
        lockedDriveDirectionY = (joyY / currentMagnitude) * lockedSpeed;
      }

      // Apply the locked speeds
      finalVelX = lockedDriveDirectionX;
      finalVelY = lockedDriveDirectionY;
    } else {
      // Normal teleop driving logic
      if (currentMagnitude > XboxInterfaceConstants.kDeadband) {
        finalVelX = joyX * DriveConstants.DRIVE_CONFIG.maxLinearVelocity();
        finalVelY = joyY * DriveConstants.DRIVE_CONFIG.maxLinearVelocity();
      }

      // Reset locks
      lockedDriveDirectionX = 0.0;
      lockedDriveDirectionY = 0.0;
      lockedSpeed = 0.0;
    }

    // Save the current state for the next loop
    wasScoring = isScoring;

    turningVelocity = Math.abs(turningVelocity) > XboxInterfaceConstants.kDeadband
        ? turningVelocity * DriveConstants.DRIVE_CONFIG.maxAngularVelocity()
        : 0.0;

    // Construct desired chassis speeds
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(finalVelX,
        finalVelY,
        turningVelocity,
        CatzRobotTracker.getInstance().getEstimatedPose().getRotation());
    // Send new chassisspeeds object to the drivetrain
    m_drivetrain.drive(chassisSpeeds);
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
