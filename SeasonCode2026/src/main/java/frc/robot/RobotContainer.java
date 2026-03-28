package frc.robot;

import java.util.Set;

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
import frc.robot.CatzSubsystems.CatzIndexer.CatzSpindexer.CatzSpindexer;
import frc.robot.CatzSubsystems.CatzIndexer.CatzYdexer.CatzYdexer;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;
import frc.robot.CatzSubsystems.CatzShooter.CatzFlywheels.CatzFlywheels;
import frc.robot.CatzSubsystems.CatzShooter.CatzHood.CatzHood;
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

    // -------------------------------------------------------------------------
    // HOARDING CONTROLS
    // -------------------------------------------------------------------------

    xboxDrv.leftBumper().whileTrue(superstructure.cmdHoardShoot());
    xboxDrv.leftBumper().onFalse(CatzSuperstructure.Instance.cmdShooterStop().alongWith(CatzSuperstructure.Instance.trackStaticHub()));

    // Hoard Toggle
    xboxDrv.rightStick().onTrue(CatzSuperstructure.Instance.toggleHoardLocation());

    // Robot Position Reset
    // Right field Corner
    xboxDrv.rightTrigger().multiPress(3, 0.4).onTrue(Commands.runOnce(() -> CatzRobotTracker.Instance.resetPose(new Pose2d(FieldConstants.getCorner(true), CatzRobotTracker.Instance.getEstimatedPose().getRotation()))));
    // Left Field Corner
    xboxDrv.leftTrigger().multiPress(3, 0.4).onTrue(Commands.runOnce(() -> CatzRobotTracker.Instance.resetPose(new Pose2d(FieldConstants.getCorner(false), CatzRobotTracker.Instance.getEstimatedPose().getRotation()))));


    // -------------------------------------------------------------------------
    // HUB SCORING CONTROLS
    // -------------------------------------------------------------------------

    xboxDrv.rightBumper().onTrue(Commands.defer(
      () -> {
        if(CatzSuperstructure.Instance.getIsScoring()){
          return CatzSuperstructure.Instance.cmdHubShoot();
        }else{
          return CatzSuperstructure.Instance.cmdShooterStop().alongWith(superstructure.trackStaticHub());
        }
      },
      Set.of(CatzTurret.Instance, CatzFlywheels.Instance, CatzHood.Instance, CatzSpindexer.Instance, CatzYdexer.Instance)));

    // xboxDrv.rightBumper().onTrue(CatzSuperstructure.Instance.cmdHubShoot());
    // xboxDrv.rightBumper().onFalse(CatzSuperstructure.Instance.cmdShooterStop().alongWith(superstructure.trackStaticHub()));

    // -------------------------------------------------------------------------
    // GLOBAL STOP CONTROL
    // -------------------------------------------------------------------------

    xboxDrv.x().onTrue(CatzSuperstructure.Instance.cmdShooterStop().alongWith(superstructure.trackStaticHub()));

    // -------------------------------------------------------------------------
    // CLIMBING CONTROL
    // -------------------------------------------------------------------------

    xboxDrv.start().multiPress(3, 0.4).onTrue(CatzSuperstructure.Instance.autoClimbCommand());

    //--------------------------------------------------------------------------
    // INTAKE
    // -------------------------------------------------------------------------
    xboxDrv.leftStick().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.toggleIntakeDeploy());
    xboxDrv.b().onTrue(CatzSuperstructure.Instance.toggleIntakeRollers());

    xboxDrv.y().onTrue(CatzSuperstructure.Instance.jiggleIntakeCommand());
    xboxDrv.y().onFalse(CatzSuperstructure.Instance.deployIntake().alongWith(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)));

    xboxDrv.povDown().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.reverseIndexers());

    // -------------------------------------------------------------------------
    // FUNCTIONAL CONTROLS with XBOX AUX
    // -------------------------------------------------------------------------
    //x on the drv controller to stop
    xboxAux.a().onTrue(CatzSuperstructure.Instance.applyFlywheelTuningSetpoint());
    // xboxAux.b().onTrue(CatzSuperstructure.Instance.applyHoodTuningSetpoint());
    // xboxAux.x().onTrue(CatzSuperstructure.Instance.applyHoodInterpolatedSetpoint());
    // xboxAux.y().onTrue(CatzSuperstructure.Instance.toggleYdexer().alongWith(CatzSuperstructure.Instance.toggleSpindexer()));

    // xboxAux.start().onTrue(CatzFlywheels.Instance.setpointCommand(Setpoint.withVoltageSetpoint(3.5)));

    // xboxAux.povUp().onTrue(CatzSuperstructure.Instance.cmdClimbReach());
    // xboxAux.povDown().onTrue(CatzSuperstructure.Instance.cmdClimbStow());

    // xboxAux.y().onTrue(superstructure.toggleManualExtendClimb());

    // xboxAux.a().onTrue(superstructure.enableClimbSoftLimit());
    // xboxAux.b().onTrue(superstructure.disableClimbSoftLimit());
    // xboxAux.x().onTrue(superstructure.resetClimbPose());
    // xboxAux.leftStick().onTrue(CatzSuperstructure.Instance.deployIntake());

    // // xboxAux.rightStick().onTrue(CatzSuperstructure.Instance.stowIntake());
    xboxAux.b().onTrue(CatzSuperstructure.Instance.toggleIntakeRollers());
    xboxAux.x().onTrue(CatzSuperstructure.Instance.toggleSpindexer());
    xboxAux.y().onTrue(CatzSuperstructure.Instance.toggleYdexer());
    xboxAux.start().onTrue(CatzSuperstructure.Instance.cmdShooterStop());
    xboxAux.leftBumper().onTrue(CatzSuperstructure.Instance.toggleHood());
    xboxAux.rightBumper().onTrue(CatzSuperstructure.Instance.toggleTurret());

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
