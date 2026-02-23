package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzIndexer.CatzSpindexer.CatzSpindexer;
import frc.robot.CatzSubsystems.CatzIndexer.CatzSpindexer.SpindexerConstants;
import frc.robot.CatzSubsystems.CatzIndexer.CatzYdexer.CatzYdexer;
import frc.robot.CatzSubsystems.CatzIndexer.CatzYdexer.YdexerConstants;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.CatzTurret;
import frc.robot.CatzSubsystems.CatzShooter.regressions.ShooterRegression;
import frc.robot.CatzSubsystems.CatzVision.ApriltagScanning.LimelightSubsystem;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TeleopDriveCmd;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.DoublePressTracker;

public class RobotContainer {
  private final CatzSuperstructure superstructure = CatzSuperstructure.Instance;
  private final CatzDrivetrain drivetrain = CatzDrivetrain.getInstance();

  private static final CommandXboxController xboxDrv = new CommandXboxController(0);
  private static final CommandXboxController xboxTest = new CommandXboxController(1);
  private static final CommandXboxController xboxFunctional = new CommandXboxController(4);

  public RobotContainer() {
    configureBindings();

    var turret = CatzTurret.Instance;
    var tracker = CatzRobotTracker.Instance;
    var vision = LimelightSubsystem.Instance;
    var regression = ShooterRegression.TUNABLE_HOOD_ANGLE_MIN;

    CatzTurret.Instance.setDefaultCommand(CatzSuperstructure.Instance.trackStaticHub());
  }

  private void configureBindings() {
    CatzDrivetrain.getInstance().setDefaultCommand(new TeleopDriveCmd(() -> xboxDrv.getLeftX(), () -> xboxDrv.getLeftY(),
        () -> xboxDrv.getRightX(), CatzDrivetrain.getInstance()));
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

    // -------------------------------------------------------------------------
    // HOARDING (Left Bumper)
    // -------------------------------------------------------------------------
    // Held: Shoot
    xboxDrv.leftBumper().whileTrue(CatzSuperstructure.Instance.cmdHoardShoot());

    // Released: Go to Standby (Keep Flywheel, Stow Hood)
    xboxDrv.leftBumper().onFalse(CatzSuperstructure.Instance.cmdHoardStandby());

    // Toggle Location
    xboxDrv.rightStick().onTrue(CatzSuperstructure.Instance.toggleHoardLocation());

    xboxDrv.rightTrigger().multiPress(2, 0.4).onTrue(Commands.runOnce(() -> CatzRobotTracker.Instance.resetPose(new Pose2d(FieldConstants.getCorner(true), CatzRobotTracker.Instance.getEstimatedPose().getRotation()))));
    xboxDrv.leftTrigger().multiPress(2, 0.4).onTrue(Commands.runOnce(() -> CatzRobotTracker.Instance.resetPose(new Pose2d(FieldConstants.getCorner(false), CatzRobotTracker.Instance.getEstimatedPose().getRotation()))));


    // -------------------------------------------------------------------------
    // HUB SCORING (Right Bumper)
    // -------------------------------------------------------------------------
    // Held: Shoot
    xboxDrv.rightBumper().whileTrue(CatzSuperstructure.Instance.cmdHubShoot());

    // Released: Go to Standby (Keep Flywheel, Stow Hood)
    xboxDrv.rightBumper().onFalse(CatzSuperstructure.Instance.cmdHubStandby());


    // -------------------------------------------------------------------------
    // GLOBAL STOP (X Button)
    // -------------------------------------------------------------------------
    xboxDrv.x().onTrue(CatzSuperstructure.Instance.cmdShooterStop().alongWith(superstructure.trackStaticHub()));

    // -------------------------------------------------------------------------
    // CLIMB (D pad Left and Right)
    // -------------------------------------------------------------------------

    DoublePressTracker.createTrigger(xboxDrv.start()).onTrue(Commands.runOnce(() -> superstructure.isClimbMode = !superstructure.isClimbMode));
    xboxDrv.povLeft().onTrue(CatzSuperstructure.Instance.alignToClimb(false));
    xboxDrv.povRight().onTrue(CatzSuperstructure.Instance.alignToClimb(true));
    // INTAKE
    // -------------------------------------------------------------------------
    xboxDrv.leftStick().onTrue(CatzSuperstructure.Instance.toggleIntakeDeploy());
    xboxDrv.b().onTrue(CatzSuperstructure.Instance.toggleIntakeRollers());

    // ---------------------Testing Controls--------------------
    // xboxTest.b().onTrue(superstructure.flywheelManualCommand());
    // xboxTest.a().onTrue(superstructure.hoodManualCommand());
    xboxTest.x().onTrue(superstructure.applyFlywheelTuningSetpoint());
    // xboxTest.y().onTrue(superstructure.applyHoodTuningSetpoint());
    xboxTest.y().onTrue(superstructure.applyHoodInterpolatedSetpoint());


    xboxTest.a().onTrue(CatzSpindexer.Instance.setpointCommand(SpindexerConstants.ON).alongWith(CatzYdexer.Instance.setpointCommand(YdexerConstants.ON)));
    // xboxTest.b().onTrue(CatzYdexer.Instance.setpointCommand(YdexerConstants.ON));

    // xboxTest.leftBumper().onTrue(superstructure.turret30Deg());
    // xboxTest.rightBumper().onTrue(superstructure.turretMinus30Deg());

    // xboxTest.povDown().onTrue(CatzIntakeDeploy.Instance.setpointCommand(Setpoint.withVoltageSetpoint(0.0)));
    // xboxTest.povUp().onTrue(CatzIntakeDeploy.Instance.setpointCommand(Setpoint.withVoltageSetpoint(0.5)));

    // xboxTest.povRight().onTrue(CatzIntakeDeploy.Instance.setCurrentPositionCommand(Units.Rotations.of(0.0)));
    // xboxTest.leftBumper().onTrue(CatzFlywheels.Instance.setpointCommand(FlywheelConstants.OFF_SETPOINT));
    // //     .alongWith(superstructure.turretTrackHubCommand()));
    // // xboxTest.b().onTrue(superstructure.interpolateHoodAngle().alongWith(superstructure.interpolateFlywheelSpeed()));
    // // xboxTest.b().onTrue(superstructure.interpolateHoodAngle()
    // // .alongWith(superstructure.interpolateShooterSpeed()).alongWPith(superstructure.turretTrackCommand()));

    // // xboxTest.leftBumper().onTrue(superstructur][\e.turret90Degrees());
    // // xboxTest.rightBumper().onTrue(superstructure.turret90DegreesMinus());

    // xboxTest.a().onTrue(superstructure.startIndexers());
    // xboxTest.x().onTrue(superstructure.stopAllShooting());

    // -------------------------------------------------------------------------
    // FUNCTIONAL CONTROLS
    // -------------------------------------------------------------------------

    xboxFunctional.leftStick().onTrue(CatzSuperstructure.Instance.toggleIntakeDeploy());
    xboxFunctional.b().onTrue(CatzSuperstructure.Instance.toggleIntakeRollers());
    xboxFunctional.x().onTrue(CatzSuperstructure.Instance.toggleSpindexer());
    xboxFunctional.y().onTrue(CatzSuperstructure.Instance.toggleYdexer());
    xboxFunctional.a().onTrue(CatzSuperstructure.Instance.applyFlywheelTuningSetpoint());
    xboxFunctional.rightBumper().onTrue(CatzSuperstructure.Instance.toggleTurret());
    xboxFunctional.leftBumper().onTrue(CatzSuperstructure.Instance.toggleHood());
    xboxFunctional.start().onTrue(CatzSuperstructure.Instance.cmdShooterStop());

  }

  public static void rumbleDrv(double val) {
    xboxDrv.setRumble(RumbleType.kBothRumble, val);
  }
}
