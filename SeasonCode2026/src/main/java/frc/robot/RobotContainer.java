package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzTurret.CatzTurret;
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
    CatzTurret.Instance.setDefaultCommand(
      superstructure.turretManualTrackCommand()
    );
    // xboxDrv.a().onTrue(superstructure.turretTrackCommand());
    xboxDrv.a().onTrue(CatzTurret.Instance.followSetpointCommand(() -> CatzTurret.Instance.calculateWrappedSetpoint(Angle.ofBaseUnits(0.0, Units.Radians))));
    xboxDrv.b().onTrue(CatzTurret.Instance.followSetpointCommand(() -> CatzTurret.Instance.calculateWrappedSetpoint(Angle.ofBaseUnits(21*Math.PI, Units.Radians))));
    xboxDrv.x().onTrue(superstructure.turretStowCommand().alongWith(superstructure.hoodFlywheelStowCommand()));
    // xboxTest.a().onTrue(superstructure.turretManualTrackCommand());
    xboxDrv.y().onTrue(new InstantCommand(()->CatzRobotTracker.Instance.resetPose(new Pose2d(0,0,new Rotation2d()))));
  }
}
