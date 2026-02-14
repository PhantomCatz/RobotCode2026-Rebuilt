package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.CatzTurret;
import frc.robot.CatzSubsystems.CatzShooter.regressions.ShooterRegression;
import frc.robot.CatzSubsystems.CatzVision.ApriltagScanning.LimelightSubsystem;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TeleopDriveCmd;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.DoublePressTracker;

public class RobotContainer {
  private final static CatzSuperstructure superstructure = CatzSuperstructure.Instance;

  private static final CommandXboxController xboxDrv = new CommandXboxController(0);
  private static final CommandXboxController xboxAux = new CommandXboxController(1)
  private static final CommandXboxController xboxTest = new CommandXboxController(2);

  public RobotContainer() {
    configureBindings();
    var turret = CatzTurret.Instance;
    var tracker = CatzRobotTracker.Instance;
    var vision = LimelightSubsystem.Instance;
    var regression = ShooterRegression.TUNABLE_HOOD_ANGLE_MIN;
  }

  private void configureBindings() {
    CatzDrivetrain.Instance.setDefaultCommand(new TeleopDriveCmd(() -> xboxDrv.getLeftX(), () -> xboxDrv.getLeftY(),
        () -> xboxDrv.getRightX(), CatzDrivetrain.Instance));
    // CatzTurret.Instance.setDefaultCommand(
    // superstructure.turretManualTrackCommand()
    DoublePressTracker.createTrigger(xboxDrv.back()).onTrue(new InstantCommand(() -> {
      if (AllianceFlipUtil.shouldFlip()) {
        CatzRobotTracker.Instance
            .resetPose(new Pose2d(CatzRobotTracker.Instance.getEstimatedPose().getTranslation(), Rotation2d.k180deg));
      } else {
        CatzRobotTracker.Instance
            .resetPose(new Pose2d(CatzRobotTracker.Instance.getEstimatedPose().getTranslation(), new Rotation2d()));
      }
    }));
    // );

    // ----------------------Shooting-----------------------
    xboxDrv.leftBumper().onTrue(superstructure.prepareForShooting());
    xboxDrv.leftBumper().onFalse(superstructure.setShootingAllowed(true));
    xboxDrv.x().onTrue(superstructure.stopAllShooting());

    // ---------------------Testing Controls--------------------
    // xboxTest.b().onTrue(superstructure.applyFlywheelTuningSetpoint().alongWith(superstructure.applyHoodTuningSetpoint())
    // .alongWith(superstructure.turretTrackHubCommand()));
    xboxTest.b().onTrue(superstructure.interpolateHoodAngle().alongWith(superstructure.interpolateFlywheelSpeed()));
    // xboxTest.b().onTrue(superstructure.interpolateHoodAngle()
    // .alongWith(superstructure.interpolateShooterSpeed()).alongWith(superstructure.turretTrackCommand()));
    xboxTest.leftBumper().onTrue(superstructure.turretTrackHubCommand());

    // xboxTest.leftBumper().onTrue(superstructure.turret90Degrees());
    // xboxTest.rightBumper().onTrue(superstructure.turret90DegreesMinus());

    xboxTest.a().onTrue(superstructure.startIndexers());
    xboxTest.x().onTrue(superstructure.stopAllShooting());


    //Manual Overide Commands
    
    xboxAux.rightBumper().toggleOnTrue(superstructure.manualCommandSwitch());

    xboxAux.a().and(() -> CatzSuperstructure.isManualCommandOn).onTrue(superstructure.hoodManualCommand().alongWith(superstructure.stopFlywheelManualCommand())
        .alongWith(superstructure.stopTurretManualCommand()));

    xboxAux.b().and(() -> CatzSuperstructure.isManualCommandOn).onTrue(superstructure.flywheelManualCommand().alongWith(superstructure.stopHoodManualCommand())
        .alongWith(superstructure.stopTurretManualCommand()));

    xboxAux.y().and(() -> CatzSuperstructure.isManualCommandOn).onTrue(superstructure.turretManualCommand().alongWith(superstructure.stopHoodManualCommand())
        .alongWith(superstructure.stopFlywheelManualCommand()));
  }

  public static void rumbleDrv(double val) {
    xboxDrv.setRumble(RumbleType.kBothRumble, val);
  }


}
