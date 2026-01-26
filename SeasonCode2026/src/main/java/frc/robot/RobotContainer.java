package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzHood.CatzHood;
import frc.robot.CatzSubsystems.CatzHood.HoodConstants;
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
    xboxDrv.start().onTrue(new InstantCommand(() -> CatzRobotTracker.Instance.resetPose(new Pose2d(0,0,new Rotation2d()))));
    // );
    // xboxDrv.a().onTrue(CatzFlywheels.Instance.followSetpointCommand(() -> {
    //   return Setpoint.withVelocitySetpoint(FlywheelConstants.SHOOTING_RPS_TUNABLE.get());
    // }));

    // xboxDrv.b().onTrue(CatzTurret.Instance.followSetpointCommand(() -> CatzTurret.Instance.calculateWrappedSetpoint(Angle.ofBaseUnits(21*Math.PI, Units.Radians))));
    // xboxDrv.x().onTrue(superstructure.turretStowCommand().alongWith(superstructure.hoodFlywheelStowCommand()));
    // xboxTest.a().onTrue(superstructure.turretManualTrackCommand());
    // xboxDrv.y().onTrue(new InstantCommand(()->CatzRobotTracker.Instance.resetPose(new Pose2d(0,0,new Rotation2d()))));
    // xboxDrv.b().onTrue(CatzIntakeRoller.Instance.setpointCommand(() -> CatzIntakeRoller.Instance.toggleIntake()));
    // xboxDrv.a().onTrue(CatzIntakeDeploy.Instance.followSetpointCommand(() -> Setpoint.withMotionMagicSetpoint(60.0)));
    // xboxDrv.leftBumper().onTrue(CatzIntakeDeploy.Instance.setCurrentPositionCommand(IntakeDeployConstants.HOME_POSITION));
    // xboxDrv.x().onTrue(CatzIntakeDeploy.Instance.followSetpointCommand(() -> Setpoint.withMotionMagicSetpoint(IntakeDeployConstants.HOME_POSITION)));
    // xboxDrv.rightBumper().onTrue(CatzIntakeDeploy.Instance.followSetpointCommand(() -> Setpoint.withVoltageSetpoint(0.0)));

    // xboxDrv.a().onTrue(CatzSpindexer.Instance.setpointCommand(()->Setpoint.withVoltageSetpoint(SpindexerConstants.SPEED.get())).alongWith(CatzYdexer.Instance.setpointCommand(()->Setpoint.withVoltageSetpoint(YdexerConstants.SPEED.get()))));
    // xboxDrv.b().onTrue(CatzSpindexer.Instance.setpointCommand(SpindexerConstants.OFF).alongWith(CatzYdexer.Instance.setpointCommand(YdexerConstants.OFF)));
    xboxTest.a().onTrue(superstructure.hoodManualCommand());
    xboxTest.b().onTrue(CatzHood.Instance.setpointCommand(HoodConstants.HOOD_TEST_SETPOINT));
    //xboxTest.y().onTrue(superstructure.intakeDeployManualCommand());
    // xboxDrv.x().onTrue(superstructure.SlapDown());
    // xboxDrv.y().onTrue(superstructure.IntakeOn());
    // xboxDrv.a().onTrue(CatzIntakeRoller.Instance.followSetpointCommand(() -> IntakeRollerConstants.OFF_SETPOINT));

  }
}
