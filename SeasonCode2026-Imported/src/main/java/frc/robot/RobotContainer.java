package frc.robot;


import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.command2.Commands;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.button.CommandGamepad;

import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TeleopDriveCmd;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.DoublePressTracker;

public class RobotContainer {
  public static final CommandGamepad xboxDrv = new CommandGamepad(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Default driving command
    CatzDrivetrain.getInstance().setDefaultCommand(new TeleopDriveCmd(() -> xboxDrv.getLeftX(), () -> xboxDrv.getLeftY(),
        () -> xboxDrv.getRightX(), CatzDrivetrain.getInstance()));

    // reset robot heading based on the current alliance color
    DoublePressTracker.createTrigger(xboxDrv.back()).onTrue(new InstantCommand(() -> {
      if (AllianceFlipUtil.shouldFlip()) {
        CatzRobotTracker.Instance
            .resetPose(new Pose2d(CatzRobotTracker.Instance.getEstimatedPose().getTranslation(), Rotation2d.k180deg));
      } else {
        CatzRobotTracker.Instance
            .resetPose(new Pose2d(CatzRobotTracker.Instance.getEstimatedPose().getTranslation(), new Rotation2d()));
      }
    }));

    //X LOCK DRIVETRAIN
    xboxDrv.povLeft().whileTrue(
    Commands.run(
        () -> CatzDrivetrain.getInstance().setXLock(),
        CatzDrivetrain.getInstance()
    )
    );
  }
}
