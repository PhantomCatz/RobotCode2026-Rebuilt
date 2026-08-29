package frc.robot;


import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TeleopDriveCmd;

public class RobotContainer {

  public static final CommandGamepad xboxDrv = new CommandGamepad(0);
  public static final CommandGamepad xboxAux = new CommandGamepad(1);

  public RobotContainer() {
    configureBindings();

  }

  private void configureBindings() {
    // Default driving command
    CatzDrivetrain.getInstance().setDefaultCommand(new TeleopDriveCmd(() -> xboxDrv.getLeftX(), () -> xboxDrv.getLeftY(),
        () -> xboxDrv.getRightX(), CatzDrivetrain.getInstance()));


    // -------------------------------------------------------------------------
    // HOARDING CONTROLS

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


    // shooting a y x start

    // -------------------------------------------------------------------------
    // MANUAL OVERRIDE
    // -------------------------------------------------------------------------


    // xboxAux.back().multiPress(2, 0.4).onTrue(Commands.runOnce(()-> CatzSuperstructure.Instance.canResetPose = ! CatzSuperstructure.Instance.canResetPose));
    // xboxAux.povUpRight().onTrue(CatzSuperstructure.Instance.resetClimbPose());

    // xboxAux.back().multiPress(2, 0.4).onTrue(CatzFlywheels.Instance.setpointCommand(Setpoint.withVoltageSetpoint(12.0)));
  }

  public static void rumbleDrv(double val) {
    xboxDrv.setRumble(RumbleType.LEFT_RUMBLE, val);
  }
}
