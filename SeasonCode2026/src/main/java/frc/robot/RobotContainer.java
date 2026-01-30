package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzShooter.CatzFlywheels.CatzFlywheels;
import frc.robot.CatzSubsystems.CatzShooter.CatzFlywheels.FlywheelConstants;
import frc.robot.CatzSubsystems.CatzShooter.CatzHood.CatzHood;
import frc.robot.CatzSubsystems.CatzShooter.CatzHood.HoodConstants;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.CatzTurret;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TeleopDriveCmd;
import frc.robot.Utilities.Setpoint;

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
    xboxTest.b().onTrue(CatzHood.Instance.setpointCommand(()->Setpoint.withMotionMagicSetpoint(HoodConstants.adjustableHoodAngle.get()))
                        .alongWith(CatzFlywheels.Instance.setpointCommand(()->Setpoint.withVelocitySetpoint(FlywheelConstants.SHOOTING_RPS_TUNABLE.get()))));
    xboxTest.a().onTrue(superstructure.startIndexers());
    xboxTest.x().onTrue(superstructure.stopAllShooting());

    System.out.println(CatzTurret.Instance);
  }
}
