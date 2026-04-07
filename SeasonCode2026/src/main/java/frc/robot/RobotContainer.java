package frc.robot;


import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzClimb.CatzClimb;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
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
    // Hoard Toggle
    xboxDrv.rightStick().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.toggleHoardLocation());

    // Robot Position Reset
    // Right field Corner
    xboxDrv.rightTrigger().multiPress(2, 0.4).onTrue(Commands.runOnce(() -> CatzRobotTracker.Instance.resetPose(new Pose2d(FieldConstants.getCorner(true), CatzRobotTracker.Instance.getEstimatedPose().getRotation()))));
    // Left Field Corner
    xboxDrv.leftTrigger().multiPress(2, 0.4).onTrue(Commands.runOnce(() -> CatzRobotTracker.Instance.resetPose(new Pose2d(FieldConstants.getCorner(false), CatzRobotTracker.Instance.getEstimatedPose().getRotation()))));

    // xboxDrv.povUp().multiPress(2, 0.4).toggleOnTrue(CatzSuperstructure.Instance.TowerSwipePosition().andThen(CatzSuperstructure.Instance.swipe()));
    // -------------------------------------------------------------------------
    // HUB SCORING CONTROLS
    // -------------------------------------------------------------------------
    // Held: Shoot

    // In RobotContainer.java constructor or a configureDefaultCommands() method

    // Turret stays in a standby tracking mode when not actively shooting

    // When nothing else is running, the turret aims at the Hub
// HOARDING (Left Bumper)
    // store the shooting commands so we can check their active state
// store the shooting commands so we can check their active state
    Command hoardShootCmd = CatzSuperstructure.Instance.cmdHoardShoot();
    Command hubShootCmd = CatzSuperstructure.Instance.cmdHubShoot();

    // bind the bumpers to toggle their respective commands
    xboxDrv.leftBumper().toggleOnTrue(hoardShootCmd);
    xboxDrv.rightBumper().toggleOnTrue(hubShootCmd);


    // create a master trigger that is true if EITHER shooting mode is running
    Trigger isShooterActive = new Trigger(() -> hoardShootCmd.isScheduled() || hubShootCmd.isScheduled());

    // ONLY run the stop and track command when both modes turn off
    isShooterActive.onFalse(
        CatzSuperstructure.Instance.cmdShooterStop()
            .alongWith(CatzSuperstructure.Instance.trackStaticHub())
    );

    // -------------------------------------------------------------------------
    // GLOBAL STOP CONTROL
    // -------------------------------------------------------------------------

    xboxDrv.x().onTrue(CatzSuperstructure.Instance.cmdShooterStop().alongWith(CatzSuperstructure.Instance.trackStaticHub()));
    //X LOCK DRIVETRAIN
    xboxDrv.povLeft().whileTrue(
    Commands.run(
        () -> CatzDrivetrain.getInstance().setXLock(), 
        CatzDrivetrain.getInstance()
    )
);
    // -------------------------------------------------------------------------
    // CLIMBING CONTROL
    // -------------------------------------------------------------------------

    // xboxDrv.start().multiPress(3, 0.4).onTrue(CatzSuperstructure.Instance.autoClimbCommand());

    //--------------------------------------------------------------------------
    // INTAKE
    // -------------------------------------------------------------------------
    xboxDrv.leftStick().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.toggleIntakeDeploy());
    xboxDrv.b().onTrue(CatzSuperstructure.Instance.toggleIntakeRollers());

    xboxDrv.y().whileTrue(CatzSuperstructure.Instance.jiggleIntakeCommand());
    xboxDrv.y().onFalse(CatzSuperstructure.Instance.deployIntake().andThen(Commands.defer(() -> {
      if(CatzSuperstructure.Instance.isIntakeOn){
        return CatzSuperstructure.Instance.intakeON();
      }else{
        return CatzSuperstructure.Instance.intakeOFF();
      }
    }, Set.of(CatzIntakeRoller.Instance))));

    xboxDrv.povDown().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.reverseIndexers());

    // -------------------------------------------------------------------------
    // FUNCTIONAL CONTROLS with XBOX AUX
    // -------------------------------------------------------------------------
    //x on the drv controller to stop
    // xboxAux.b().onTrue(CatzSuperstructure.Instance.applyHoodInterpolatedSetpoint());
    // xboxAux.x().onTrue(CatzSuperstructure.Instance.trackHoardLocation());

    // xboxAux.y().onTrue(CatzSuperstructure.Instance.toggleYdexer().alongWith(CatzSuperstructure.Instance.toggleSpindexer()));
    // xboxAux.x().onTrue(CatzSuperstructure.Instance.applyHoodInterpolatedSetpoint());

    // xboxAux.start().onTrue(CatzFlywheels.Instance.setpointCommand(Setpoint.withVoltageSetpoint(3.5)));

    // xboxAux.povUp().onTrue(CatzSuperstructure.Instance.cmdClimbReach());
    // xboxAux.povDown().onTrue(CatzSuperstructure.Instance.cmdClimbStow());

    // xboxAux.y().onTrue(superstructure.toggleManualExtendClimb());

    // xboxAux.a().onTrue(superstructure.enableClimbSoftLimit());
    // xboxAux.b().onTrue(superstructure.disableClimbSoftLimit());
    // xboxAux.x().onTrue(superstructure.resetClimbPose());
    xboxAux.b().onTrue(CatzSuperstructure.Instance.toggleIntakeRollers());
    xboxAux.x().onTrue(CatzSuperstructure.Instance.toggleSpindexer());
    xboxAux.y().onTrue(CatzSuperstructure.Instance.toggleYdexer());
    xboxAux.leftBumper().onTrue(CatzSuperstructure.Instance.toggleHood());
    xboxAux.a().onTrue(CatzSuperstructure.Instance.applyFlywheelTuningSetpoint());
    xboxAux.start().onTrue(CatzSuperstructure.Instance.cmdShooterStop());
    xboxAux.rightBumper().onTrue(CatzSuperstructure.Instance.toggleTurret());

    // shooting a y x start

    // xboxAux.leftStick().onTrue(CatzSuperstructure.Instance.deployIntake());
    // xboxAux.rightStick().onTrue(CatzSuperstructure.Instance.stowIntake());
    // -------------------------------------------------------------------------
    // MANUAL OVERRIDE
    // -------------------------------------------------------------------------


    xboxAux.povUp().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.toggleManualExtendClimb());
    xboxAux.povDown().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.toggleManualHood());
    xboxAux.povLeft().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.toggleManualTurret());
    xboxAux.povRight().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.toggleManualDeploy());


    xboxAux.back().multiPress(2, 0.4).onTrue(Commands.runOnce(()-> CatzSuperstructure.Instance.canResetPose = ! CatzSuperstructure.Instance.canResetPose));
    xboxAux.povUpRight().onTrue(CatzSuperstructure.Instance.resetClimbPose());
    xboxAux.povDownLeft().onTrue(CatzSuperstructure.Instance.resetHoodPose());
    xboxAux.povUpLeft().onTrue(CatzSuperstructure.Instance.resetTurretPose());
    xboxAux.povDownRight().onTrue(CatzSuperstructure.Instance.resetDeployPose());

  }

  public static void rumbleDrv(double val) {
    xboxDrv.setRumble(RumbleType.kBothRumble, val);
  }
}
