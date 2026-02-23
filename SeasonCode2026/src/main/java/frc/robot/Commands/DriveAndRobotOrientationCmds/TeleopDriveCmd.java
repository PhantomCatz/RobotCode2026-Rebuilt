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
  private double m_headingAndVelocity_X;
  private double m_headingAndVelocity_Y;
  private double turningVelocity;

  private ChassisSpeeds chassisSpeeds;

  private double lockedDriveDirectionX = 0.0;
  private double lockedDriveDirectionY = 0.0;
  private static final double SHOOTING_JOYSTICK_DEADBAND = 0.8;

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
public void initialize() {}

  // --------------------------------------------------------------------------------------
  //
  // Execute
  //
  // --------------------------------------------------------------------------------------
  @Override
  public void execute() {
    // Obtain realtime joystick inputs with supplier methods
    m_headingAndVelocity_X = -m_headingPctOutput_Y.get(); //Raw accel
    m_headingAndVelocity_Y = -m_headingPctOutput_X.get();
    turningVelocity        = -m_angVelocityPctOutput.get(); // alliance flip shouldn't change for turing speed when switching alliances

    // Flip Directions for left joystick if alliance is red

    if (DriverStation.getAlliance().get() == Alliance.Red) {
      m_headingAndVelocity_X = -m_headingAndVelocity_X;
      m_headingAndVelocity_Y = -m_headingAndVelocity_Y;
    }

    boolean isScoring = CatzSuperstructure.Instance.getIsScoring();
    double currentMagnitude = Math.hypot(m_headingAndVelocity_X, m_headingAndVelocity_Y);

    double finalVelX = 0.0;
    double finalVelY = 0.0;

    if(isScoring){
      if(currentMagnitude > SHOOTING_JOYSTICK_DEADBAND){
        lockedDriveDirectionX = (m_headingAndVelocity_X / currentMagnitude) * DriveConstants.MAX_SHOOT_WHILE_MOVE_VELOCITY;
        lockedDriveDirectionY = (m_headingAndVelocity_Y / currentMagnitude) * DriveConstants.MAX_SHOOT_WHILE_MOVE_VELOCITY;
      }

      finalVelX = lockedDriveDirectionX;
      finalVelY = lockedDriveDirectionY;
    }else{
      lockedDriveDirectionX = 0.0;
      lockedDriveDirectionY = 0.0;

      if (currentMagnitude > XboxInterfaceConstants.kDeadband) {
        finalVelX = m_headingAndVelocity_X * DriveConstants.DRIVE_CONFIG.maxLinearVelocity();
        finalVelY = m_headingAndVelocity_Y * DriveConstants.DRIVE_CONFIG.maxLinearVelocity();
      }
    }

    turningVelocity =
        Math.abs(turningVelocity) > XboxInterfaceConstants.kDeadband
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
  public void end(boolean interrupted) {}

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
