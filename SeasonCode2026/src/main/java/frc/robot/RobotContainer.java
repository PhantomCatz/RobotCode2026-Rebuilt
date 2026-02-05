package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
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
    // );
    // xboxDrv.a().onTrue(CatzFlywheels.Instance.followSetpointCommand(() -> {
    //   return Setpoint.withVelocitySetpoint(FlywheelConstants.SHOOTING_RPS_TUNABLE.get());
    // }));
    // xboxTest.b().onTrue(CatzHood.Instance.setpointCommand(()->Setpoint.withMotionMagicSetpoint(HoodConstants.adjustableHoodAngle.get()))
    //                     .alongWith(CatzFlywheels.Instance.setpointCommand(()->Setpoint.withVelocitySetpoint(FlywheelConstants.SHOOTING_RPS_TUNABLE.get()))));

    xboxTest.a().whileTrue(new RunCommand(() -> superstructure.hoodManualCommand().execute()));
    xboxTest.b().whileTrue(new RunCommand(() -> superstructure.intakeDeployManualCommand().execute()));
    xboxTest.x().whileTrue(new RunCommand(() -> superstructure.flywheelManualCommand().execute()));
    xboxTest.y().onTrue(superstructure.stopAllShooting());

    xboxTest.start().onTrue(new InstantCommand(() -> CatzRobotTracker.Instance.resetPose(new Pose2d(0,0,new Rotation2d()))));
    //new Trigger(() -> xboxTest.getLeftY() < -0.1).whileTrue(superstructure.hoodManualCommand());
  }
}
