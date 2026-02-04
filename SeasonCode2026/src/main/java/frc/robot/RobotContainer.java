package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzIntakeDeploy.CatzIntakeDeploy;
import frc.robot.CatzSubsystems.CatzIntakeDeploy.IntakeDeployConstants;
import frc.robot.CatzSubsystems.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TeleopDriveCmd;

public class RobotContainer {
  private final CatzSuperstructure superstructure = CatzSuperstructure.Instance;

  private final CommandXboxController xboxDrv = new CommandXboxController(0);
  private final CommandXboxController xboxTest = new CommandXboxController(1);

  public RobotContainer(){
    configureBindings();
  }
  private void configureBindings() {
    CatzDrivetrain.Instance.setDefaultCommand(new TeleopDriveCmd(() -> xboxDrv.getLeftX(), () -> xboxDrv.getLeftY(), () -> xboxDrv.getRightX(), CatzDrivetrain.Instance));
    // CatzTurret.Instance.setDefaultCommand(
    //   superstructure.turretManualTrackCommand()
    //xboxDrv.start().onTrue(new InstantCommand(() -> CatzRobotTracker.Instance.resetPose(new Pose2d(0,0,new Rotation2d()))));
    // );
    // xboxDrv.a().onTrue(CatzFlywheels.Instance.followSetpointCommand(() -> {
    //   return Setpoint.withVelocitySetpoint(FlywheelConstants.SHOOTING_RPS_TUNABLE.get());
    // }));
    // xboxTest.b().onTrue(CatzHood.Instance.setpointCommand(()->Setpoint.withMotionMagicSetpoint(HoodConstants.adjustableHoodAngle.get()))
    //                     .alongWith(CatzFlywheels.Instance.setpointCommand(()->Setpoint.withVelocitySetpoint(FlywheelConstants.SHOOTING_RPS_TUNABLE.get()))));
    xboxDrv.a().onTrue(CatzIntakeDeploy.Instance.setpointCommand(IntakeDeployConstants.Zero));
    xboxDrv.b().onTrue(CatzIntakeDeploy.Instance.setpointCommand(IntakeDeployConstants.Sixty));
    xboxDrv.y().onTrue(CatzIntakeDeploy.Instance.setpointCommand(IntakeDeployConstants.HoldDown));
    xboxDrv.x().onTrue(CatzIntakeRoller.Instance.setpointCommand(() -> CatzIntakeRoller.Instance.toggleIntake()));

    // xboxTest.b().onTrue(superstructure.stopAllShooting());


  }
}
