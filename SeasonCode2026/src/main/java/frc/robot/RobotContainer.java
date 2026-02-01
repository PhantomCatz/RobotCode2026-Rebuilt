package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.CatzTurret;
import frc.robot.CatzSubsystems.CatzShooter.regressions.ShooterRegression;
import frc.robot.CatzSubsystems.CatzVision.ApriltagScanning.LimelightSubsystem;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TeleopDriveCmd;

public class RobotContainer {
  private final CatzSuperstructure superstructure = CatzSuperstructure.Instance;

  private final CommandXboxController xboxDrv = new CommandXboxController(0);
  private final CommandXboxController xboxTest = new CommandXboxController(1);

  public RobotContainer(){
    configureBindings();

    var turret = CatzTurret.Instance;
    var tracker = CatzRobotTracker.Instance;
    var vision = LimelightSubsystem.Instance;
    var regression = ShooterRegression.TUNABLE_HOOD_ANGLE_MIN;
  }
  private void configureBindings() {
    CatzDrivetrain.Instance.setDefaultCommand(new TeleopDriveCmd(() -> xboxDrv.getLeftX(), () -> xboxDrv.getLeftY(), () -> xboxDrv.getRightX(), CatzDrivetrain.Instance));
    // CatzTurret.Instance.setDefaultCommand(
    //   superstructure.turretManualTrackCommand()
    xboxDrv.start().onTrue(new InstantCommand(() -> CatzRobotTracker.Instance.resetPose(new Pose2d(FieldConstants.HUB_LOCATION, new Rotation2d()))));
    // );
    // xboxDrv.a().onTrue(CatzFlywheels.Instance.followSetpointCommand(() -> {
    //   return Setpoint.withVelocitySetpoint(FlywheelConstants.SHOOTING_RPS_TUNABLE.get());
    // }));

    // xboxTest.b().onTrue(superstructure.interpolateHoodAngle()
    //                     .alongWith(superstructure.interpolateShooterSpeed()).alongWith(superstructure.turretTrackCommand()));
    xboxTest.b().onTrue(superstructure.turretTrackCommand());
    xboxTest.leftBumper().onTrue(superstructure.turret90Degrees());
    xboxTest.rightBumper().onTrue(superstructure.turret90DegreesMinus());

    xboxTest.a().onTrue(superstructure.startIndexers());
    xboxTest.x().onTrue(superstructure.stopAllShooting());
  }
}
