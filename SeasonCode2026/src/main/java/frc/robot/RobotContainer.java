package frc.robot;


import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeDeploy.CatzIntakeDeploy;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TeleopDriveCmd;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.DoublePressTracker;

public class RobotContainer {
  private final CatzSuperstructure superstructure = CatzSuperstructure.Instance;

  public static final CommandXboxController xboxDrv = new CommandXboxController(0);
  public static final CommandXboxController xboxAux = new CommandXboxController(1);

  public RobotContainer() {
    configureBindings();

    var superstructure = CatzSuperstructure.Instance;
    System.out.println(superstructure);

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
    xboxDrv.b().onTrue(CatzSuperstructure.Instance.toggleIntakeRollers());

    // Robot Position Reset
    // Right field Corner
    xboxDrv.rightTrigger().multiPress(2, 0.4).onTrue(Commands.runOnce(() -> CatzRobotTracker.Instance.resetPose(new Pose2d(FieldConstants.getCorner(true), CatzRobotTracker.Instance.getEstimatedPose().getRotation()))));
    // Left Field Corner
    xboxDrv.leftTrigger().multiPress(2, 0.4).onTrue(Commands.runOnce(() -> CatzRobotTracker.Instance.resetPose(new Pose2d(FieldConstants.getCorner(false), CatzRobotTracker.Instance.getEstimatedPose().getRotation()))));

    xboxDrv.povRight().multiPress(2, 0.4).toggleOnTrue(CatzSuperstructure.Instance.TowerSwipePosition().andThen(CatzSuperstructure.Instance.swipe()).until(() -> xboxDrv.x().getAsBoolean())
    .beforeStarting(() -> SmartDashboard.putBoolean("Swiping?", true))
    .finallyDo(() -> SmartDashboard.putBoolean("Swiping?", false)));

    // -------------------------------------------------------------------------
    // GLOBAL STOP CONTROL
    // -------------------------------------------------------------------------

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
    xboxDrv.b().onTrue(CatzSuperstructure.Instance.toggleIntakeRollers());

    xboxDrv.y().onTrue(Commands.runOnce(() -> CatzIntakeDeploy.Instance.setGainsPV(10.5, 1.5)));
    xboxDrv.y().whileTrue(CatzSuperstructure.Instance.jiggleIntakeCommand());
    xboxDrv.y().onFalse((Commands.runOnce(() -> CatzIntakeDeploy.Instance.setGainsPV(10.5, 2)))
               .alongWith(CatzSuperstructure.Instance.deployIntake().andThen(Commands.defer(() -> {
      if(CatzSuperstructure.Instance.isIntakeOn){
        return CatzSuperstructure.Instance.intakeON();
      }else{
        return CatzSuperstructure.Instance.intakeOFF();
      }
    }, Set.of(CatzIntakeRoller.Instance)))));

    xboxDrv.povUp().onTrue(superstructure.incrementScorePosition());
    xboxDrv.povDown().onTrue(superstructure.decrementScorePosition());
    xboxDrv.a().onTrue(superstructure.armLX());
    xboxDrv.x().onTrue(superstructure.armStow());

    // -------------------------------------------------------------------------
    // FUNCTIONAL CONTROLS with XBOX AUX
    // -------------------------------------------------------------------------

    xboxAux.a().onTrue(superstructure.armHold());
    xboxAux.x().onTrue(superstructure.armAlgae());

    xboxAux.y().onTrue(superstructure.toggleManualMantaPivot());

    xboxAux.start().multiPress(2, 0.4).onTrue(superstructure.enableClimbSoftLimit().alongWith(superstructure.resetClimbPose()));
    xboxAux.back().multiPress(2, 0.4).onTrue(superstructure.disableClimbSoftLimit());

    xboxAux.b().onTrue(CatzSuperstructure.Instance.toggleIntakeRollers());

    xboxAux.leftStick().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.cmdClimbStow().andThen(Commands.waitSeconds(0.5).andThen(CatzSuperstructure.Instance.deployIntake())));
    xboxAux.rightStick().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.stowIntake().andThen(Commands.waitSeconds(0.5)).andThen(CatzSuperstructure.Instance.cmdClimbReach()));

    // xboxAux.leftTrigger().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.toggleDefenseMode());
    xboxAux.leftTrigger().onTrue(CatzSuperstructure.Instance.toggleDefenseMode());
    xboxAux.leftTrigger().onFalse(CatzSuperstructure.Instance.toggleDefenseMode());
    // -------------------------------------------------------------------------
    // MANUAL OVERRIDE
    // -------------------------------------------------------------------------

    xboxAux.povUp().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.toggleManualExtendClimb());
    xboxAux.povRight().multiPress(2, 0.4).onTrue(CatzSuperstructure.Instance.toggleManualDeploy());


    // xboxAux.back().multiPress(2, 0.4).onTrue(Commands.runOnce(()-> CatzSuperstructure.Instance.canResetPose = ! CatzSuperstructure.Instance.canResetPose));
    // xboxAux.povUpRight().onTrue(CatzSuperstructure.Instance.resetClimbPose());
    xboxAux.povDownRight().onTrue(CatzSuperstructure.Instance.resetDeployPose());
    // xboxAux.back().multiPress(2, 0.4).onTrue(CatzFlywheels.Instance.setpointCommand(Setpoint.withVoltageSetpoint(12.0)));
  }

  public static void rumbleDrv(double val) {
    xboxDrv.setRumble(RumbleType.kBothRumble, val);
  }
}
