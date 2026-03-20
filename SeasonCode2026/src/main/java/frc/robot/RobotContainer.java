package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzClimb.CatzClimb;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.CatzTurret;
import frc.robot.CatzSubsystems.CatzShooter.regressions.ShooterRegression;
import frc.robot.CatzSubsystems.CatzVision.ApriltagScanning.LimelightSubsystem;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TeleopDriveCmd;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.DoublePressTracker;

public class RobotContainer {
  private final CatzSuperstructure superstructure = CatzSuperstructure.Instance;

  public static final CommandXboxController xboxDrv = new CommandXboxController(0);
  public static final CommandXboxController xboxAux = new CommandXboxController(1);

  public RobotContainer() {
    configureBindings();

    var turret = CatzTurret.Instance;
    var tracker = CatzRobotTracker.Instance;
    var vision = LimelightSubsystem.Instance;
    var climb = CatzClimb.Instance;
    var superstructure = CatzSuperstructure.Instance;
    System.out.println(superstructure);
    var regression = ShooterRegression.TUNABLE_HOOD_ANGLE_MIN;

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
    xboxDrv.leftBumper().whileTrue(superstructure.cmdHoardShoot());

    // Released: Go to Standby (Keep Flywheel, Stow Hood)
    xboxDrv.leftBumper().onFalse(CatzSuperstructure.Instance.cmdShooterStop().alongWith(CatzSuperstructure.Instance.trackStaticHub()).alongWith(Commands.runOnce(() -> DriveConstants.MAX_SHOOT_WHILE_MOVE_VELOCITY = 2.0)));

    // Toggle Location
    xboxDrv.rightStick().onTrue(CatzSuperstructure.Instance.toggleHoardLocation());

    // xboxDrv.rightTrigger().onTrue(Commands.runOnce(() -> DriveConstants.MAX_SHOOT_WHILE_MOVE_VELOCITY += 1.0));
    // xboxDrv.leftTrigger().onTrue(Commands.runOnce(() -> DriveConstants.MAX_SHOOT_WHILE_MOVE_VELOCITY -= 1.0));
    // xboxDrv.rightTrigger().onTrue(CatzSuperstructure.Instance.deployIntake());
    // xboxDrv.leftTrigger().onTrue(CatzSuperstructure.Instance.stowIntake());
    xboxDrv.rightTrigger().multiPress(3, 0.4).onTrue(Commands.runOnce(() -> CatzRobotTracker.Instance.resetPose(new Pose2d(FieldConstants.getCorner(true), CatzRobotTracker.Instance.getEstimatedPose().getRotation()))));
    xboxDrv.leftTrigger().multiPress(3, 0.4).onTrue(Commands.runOnce(() -> CatzRobotTracker.Instance.resetPose(new Pose2d(FieldConstants.getCorner(false), CatzRobotTracker.Instance.getEstimatedPose().getRotation()))));


    // -------------------------------------------------------------------------
    // HUB SCORING (Right Bumper)
    // -------------------------------------------------------------------------
    // Held: Shoot
    xboxDrv.rightBumper().whileTrue(CatzSuperstructure.Instance.cmdHubShoot());


    xboxDrv.rightBumper().onFalse(CatzSuperstructure.Instance.cmdShooterStop().alongWith(superstructure.trackStaticHub()).alongWith(Commands.runOnce(() -> DriveConstants.MAX_SHOOT_WHILE_MOVE_VELOCITY = 2.0)));

    // xboxDrv.a().onTrue(CatzSuperstructure.Instance.jiggleIntakeCommand());
    // xboxDrv.a().onFalse(CatzSuperstructure.Instance.deployIntake());
    // -------------------------------------------------------------------------
    // GLOBAL STOP (X Button)
    // -------------------------------------------------------------------------
    xboxDrv.x().onTrue(CatzSuperstructure.Instance.cmdShooterStop().alongWith(superstructure.trackStaticHub()));

    // -------------------------------------------------------------------------
    // CLIMB (D pad Left and Right)
    // -------------------------------------------------------------------------
    DoublePressTracker.createTrigger(xboxDrv.start()).onTrue(Commands.runOnce(() -> superstructure.isClimbMode = !superstructure.isClimbMode)
                                                              .alongWith(superstructure.trackTower()));

    // xboxDrv.povRight().onTrue(CatzSuperstructure.Instance.autoClimbCommand());
    // INTAKE
    // -------------------------------------------------------------------------
    xboxDrv.povRight().onTrue(CatzSuperstructure.Instance.toggleIntakeDeploy());
    xboxDrv.b().onTrue(CatzSuperstructure.Instance.toggleIntakeRollers());

    xboxDrv.y().onTrue(CatzSuperstructure.Instance.jiggleIntakeCommand());
    xboxDrv.y().onFalse(CatzSuperstructure.Instance.deployIntake().alongWith(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)));

    xboxDrv.povDown().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.reverseIndexers());

    // -------------------------------------------------------------------------
    // FUNCTIONAL CONTROLS
    // -------------------------------------------------------------------------
    //x on the drv controller to stop
    xboxAux.a().onTrue(CatzSuperstructure.Instance.applyFlywheelTuningSetpoint());
    xboxAux.b().onTrue(CatzSuperstructure.Instance.applyHoodTuningSetpoint());
    xboxAux.x().onTrue(CatzSuperstructure.Instance.applyHoodInterpolatedSetpoint());
    xboxAux.y().onTrue(CatzSuperstructure.Instance.toggleYdexer().alongWith(CatzSuperstructure.Instance.toggleSpindexer()));

    // xboxAux.leftStick().onTrue(CatzSuperstructure.Instance.deployIntake());
    // xboxAux.b().onTrue(CatzSuperstructure.Instance.toggleIntakeRollers());
    // xboxAux.rightStick().onTrue(CatzSuperstructure.Instance.stowIntake());
    // xboxAux.x().onTrue(CatzSuperstructure.Instance.toggleSpindexer());
    // xboxAux.y().onTrue(CatzSuperstructure.Instance.toggleYdexer());
    // xboxAux.start().onTrue(CatzSuperstructure.Instance.cmdShooterStop());
    // xboxAux.leftBumper().onTrue(CatzSuperstructure.Instance.toggleHood());
    // xboxAux.rightBumper().onTrue(CatzSuperstructure.Instance.toggleTurret());

    // -------------------------------------------------------------------------
    // MANUAL OVERRIDE
    // -------------------------------------------------------------------------


    // xboxAux.povUp().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.toggleManualExtendClimb());
    // xboxAux.povDown().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.toggleManualHood());
    // xboxAux.povLeft().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.toggleManualTurret());
    // xboxAux.povRight().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.toggleManualDeploy());


    // xboxAux.back().multiPress(2, 0.4).onTrue(Commands.runOnce(()-> CatzSuperstructure.Instance.canResetPose = ! CatzSuperstructure.Instance.canResetPose));
    // xboxAux.povUpRight().onTrue(CatzSuperstructure.Instance.resetClimbPose());
    // xboxAux.povDownLeft().onTrue(CatzSuperstructure.Instance.resetHoodPose());
    // xboxAux.povUpLeft().onTrue(CatzSuperstructure.Instance.resetTurretPose());
    // xboxAux.povDownRight().onTrue(CatzSuperstructure.Instance.resetDeployPose());

  }

  public static void rumbleDrv(double val) {
    xboxDrv.setRumble(RumbleType.kBothRumble, val);
  }
}
