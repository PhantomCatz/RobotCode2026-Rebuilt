package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  private final CatzDrivetrain drivetrain = CatzDrivetrain.getInstance();

  public static final CommandXboxController xboxDrv = new CommandXboxController(0);
  public static final CommandXboxController xboxTest = new CommandXboxController(1);
  public static final CommandXboxController xboxFunctional = new CommandXboxController(4);

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

    // ---------------------Testing Controls--------------------

    xboxTest.a().onTrue(Commands.runOnce(() -> CatzRobotTracker.Instance.resetPose(new Pose2d(new Translation2d(2.0, 6.0), new Rotation2d())))); // top left 0
    xboxTest.b().onTrue(Commands.runOnce(() -> CatzRobotTracker.Instance.resetPose(new Pose2d(new Translation2d(6.0, 6.0), new Rotation2d())))); // mid top left 1
    xboxTest.x().onTrue(Commands.runOnce(() -> CatzRobotTracker.Instance.resetPose(new Pose2d(new Translation2d(10.0, 6.0), new Rotation2d())))); // mid top right 2
    xboxTest.y().onTrue(Commands.runOnce(() -> CatzRobotTracker.Instance.resetPose(new Pose2d(new Translation2d(14.0, 6.0), new Rotation2d())))); // top right 3
    xboxTest.povUp().onTrue(Commands.runOnce(() -> CatzRobotTracker.Instance.resetPose(new Pose2d(new Translation2d(2.0, 2.0), new Rotation2d())))); // bot left 4
    xboxTest.povRight().onTrue(Commands.runOnce(() -> CatzRobotTracker.Instance.resetPose(new Pose2d(new Translation2d(6.0, 2.0), new Rotation2d())))); // mid bot left 5
    xboxTest.povDown().onTrue(Commands.runOnce(() -> CatzRobotTracker.Instance.resetPose(new Pose2d(new Translation2d(10.0, 2.0), new Rotation2d())))); // mid bot right 6
    xboxTest.povLeft().onTrue(Commands.runOnce(() -> CatzRobotTracker.Instance.resetPose(new Pose2d(new Translation2d(14.0, 2.0), new Rotation2d())))); // bot right 7


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
    //x on the drv controller to stop
    xboxFunctional.leftStick().onTrue(CatzSuperstructure.Instance.deployIntake());
    xboxFunctional.b().onTrue(CatzSuperstructure.Instance.toggleIntakeRollers());
    xboxFunctional.rightStick().onTrue(CatzSuperstructure.Instance.stowIntake());
    xboxFunctional.x().onTrue(CatzSuperstructure.Instance.toggleSpindexer());
    xboxFunctional.y().onTrue(CatzSuperstructure.Instance.toggleYdexer());
    xboxFunctional.a().onTrue(CatzSuperstructure.Instance.applyFlywheelTuningSetpoint());
    xboxFunctional.start().onTrue(CatzSuperstructure.Instance.cmdShooterStop());
    xboxFunctional.leftBumper().onTrue(CatzSuperstructure.Instance.toggleHood());
    xboxFunctional.rightBumper().onTrue(CatzSuperstructure.Instance.toggleTurret());


  }

  public static void rumbleDrv(double val) {
    xboxDrv.setRumble(RumbleType.kBothRumble, val);
  }
}
