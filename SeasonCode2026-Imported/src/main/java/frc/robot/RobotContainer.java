package frc.robot;


import java.util.Set;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.Trigger;
import org.wpilib.driverstation.Gamepad;

import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzClimb.CatzClimb;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeDeploy.CatzIntakeDeploy;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.CatzTurret;
import frc.robot.CatzSubsystems.CatzShooter.regressions.ShooterRegression;
import frc.robot.CatzSubsystems.CatzVision.ApriltagScanning.LimelightSubsystem;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TeleopDriveCmd;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.DoublePressTracker;

public class RobotContainer {
  private final CatzSuperstructure superstructure = CatzSuperstructure.Instance;

  public static final CommandGamepad xboxDrv = new CommandGamepad(0);
  public static final CommandGamepad xboxAux = new CommandGamepad(1);

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

    xboxDrv.povUp().multiPress(2, 0.4).toggleOnTrue(CatzSuperstructure.Instance.TowerSwipePosition().andThen(CatzSuperstructure.Instance.swipe()).until(() -> xboxDrv.button(Gamepad.Button.WEST_FACE).getAsBoolean())
    .beforeStarting(() -> SmartDashboard.putBoolean("Swiping?", true))
    .finallyDo(() -> SmartDashboard.putBoolean("Swiping?", false)));

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

    xboxDrv.button(Gamepad.Button.WEST_FACE).onTrue(CatzSuperstructure.Instance.cmdShooterStop().alongWith(CatzSuperstructure.Instance.trackStaticHub()).alongWith(CatzSuperstructure.Instance.intakeOFF()));
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

    xboxDrv.start().multiPress(3, 0.4).onTrue(CatzSuperstructure.Instance.autoClimbCommand());

    //--------------------------------------------------------------------------
    // INTAKE
    // -------------------------------------------------------------------------
    xboxDrv.leftStick().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.toggleIntakeDeploy());
    xboxDrv.button(Gamepad.Button.EAST_FACE).onTrue(CatzSuperstructure.Instance.toggleIntakeRollers());

    xboxDrv.button(Gamepad.Button.NORTH_FACE).onTrue(Commands.runOnce(() -> CatzIntakeDeploy.Instance.setGainsPV(10.5, 1.5)));
    xboxDrv.button(Gamepad.Button.NORTH_FACE).whileTrue(CatzSuperstructure.Instance.jiggleIntakeCommand());
    xboxDrv.button(Gamepad.Button.NORTH_FACE).onFalse((Commands.runOnce(() -> CatzIntakeDeploy.Instance.setGainsPV(10.5, 2)))
               .alongWith(CatzSuperstructure.Instance.deployIntake().andThen(Commands.defer(() -> {
      if(CatzSuperstructure.Instance.isIntakeOn){
        return CatzSuperstructure.Instance.intakeON();
      }else{
        return CatzSuperstructure.Instance.intakeOFF();
      }
    }, Set.of(CatzIntakeRoller.Instance)))));

    xboxDrv.povDown().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.reverseIndexers());

    // -------------------------------------------------------------------------
    // FUNCTIONAL CONTROLS with XBOX AUX
    // -------------------------------------------------------------------------
    //x on the drv controller to stop
    // xboxAux.button(Gamepad.Button.EAST_FACE).onTrue(CatzSuperstructure.Instance.applyHoodInterpolatedSetpoint());
    // xboxAux.button(Gamepad.Button.WEST_FACE).onTrue(CatzSuperstructure.Instance.trackHoardLocation());

    // xboxAux.button(Gamepad.Button.NORTH_FACE).onTrue(CatzSuperstructure.Instance.toggleYdexer().alongWith(CatzSuperstructure.Instance.toggleSpindexer()));
    // xboxAux.button(Gamepad.Button.WEST_FACE).onTrue(CatzSuperstructure.Instance.applyHoodInterpolatedSetpoint());

    // xboxAux.start().onTrue(CatzFlywheels.Instance.setpointCommand(Setpoint.withVoltageSetpoint(3.5)));

    // xboxAux.povUp().onTrue(CatzSuperstructure.Instance.cmdClimbReach());
    // xboxAux.povDown().onTrue(CatzSuperstructure.Instance.cmdClimbStow());

    // xboxAux.button(Gamepad.Button.NORTH_FACE).onTrue(superstructure.toggleManualExtendClimb());

    xboxAux.start().multiPress(2, 0.4).onTrue(superstructure.enableClimbSoftLimit().alongWith(superstructure.resetClimbPose()));
    xboxAux.back().multiPress(2, 0.4).onTrue(superstructure.disableClimbSoftLimit());

    xboxAux.button(Gamepad.Button.EAST_FACE).onTrue(CatzSuperstructure.Instance.shotBlockerOut());
    xboxAux.button(Gamepad.Button.WEST_FACE).onTrue(CatzSuperstructure.Instance.shotBlockerHome());

    // xboxAux.b().onTrue(CatzSuperstructure.Instance.toggleIntakeRollers());
    // xboxAux.x().onTrue(CatzSuperstructure.Instance.toggleSpindexer());
    xboxAux.button(Gamepad.Button.NORTH_FACE).onTrue(CatzSuperstructure.Instance.toggleYdexer());
    xboxAux.leftBumper().onTrue(CatzSuperstructure.Instance.toggleHood());
    xboxAux.button(Gamepad.Button.SOUTH_FACE).onTrue(CatzSuperstructure.Instance.applyFlywheelTuningSetpoint());
    xboxAux.start().onTrue(CatzSuperstructure.Instance.cmdShooterStop());
    xboxAux.rightBumper().onTrue(CatzSuperstructure.Instance.toggleTurret());

    // shooting a y x start

    xboxAux.leftStick().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.cmdClimbStow().andThen(Commands.waitSeconds(0.5).andThen(CatzSuperstructure.Instance.deployIntake())));
    xboxAux.rightStick().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.stowIntake().andThen(Commands.waitSeconds(0.5)).andThen(CatzSuperstructure.Instance.cmdClimbReach()));

    xboxAux.rightTrigger().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.trackOpposingHub());
    // xboxAux.leftTrigger().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.toggleDefenseMode());
    xboxAux.leftTrigger().onTrue(CatzSuperstructure.Instance.toggleDefenseMode());
    xboxAux.leftTrigger().onFalse(CatzSuperstructure.Instance.toggleDefenseMode());
    // -------------------------------------------------------------------------
    // MANUAL OVERRIDE
    // -------------------------------------------------------------------------


    xboxAux.povUp().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.toggleManualExtendClimb());
    xboxAux.povDown().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.toggleManualBlocker());
    xboxAux.povLeft().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.toggleManualTurret());
    xboxAux.povRight().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.toggleManualDeploy());


    // xboxAux.back().multiPress(2, 0.4).onTrue(Commands.runOnce(()-> CatzSuperstructure.Instance.canResetPose = ! CatzSuperstructure.Instance.canResetPose));
    // xboxAux.povUpRight().onTrue(CatzSuperstructure.Instance.resetClimbPose());
    xboxAux.povDownLeft().onTrue(CatzSuperstructure.Instance.resetHoodPose());
    xboxAux.povUpLeft().onTrue(CatzSuperstructure.Instance.resetTurretPose());
    xboxAux.povDownRight().onTrue(CatzSuperstructure.Instance.resetDeployPose());
    // xboxAux.back().multiPress(2, 0.4).onTrue(CatzFlywheels.Instance.setpointCommand(Setpoint.withVoltageSetpoint(12.0)));
  }

  public static void rumbleDrv(double val) {
    xboxDrv.setRumble(RumbleType.LEFT_RUMBLE, val);
    xboxDrv.setRumble(RumbleType.RIGHT_RUMBLE, val);
  }
}
