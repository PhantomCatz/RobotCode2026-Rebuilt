package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntake;
import frc.robot.CatzSubsystems.CatzIntake.IntakeConstants;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TeleopDriveCmd;

public class RobotContainer {
  private final CatzSuperstructure superstructure = CatzSuperstructure.Instance;
  private final CatzDrivetrain drivetrain = CatzDrivetrain.getInstance();

  public static final CommandXboxController xboxDrv = new CommandXboxController(0);

  public RobotContainer() {
    System.out.println("Drivetrain in RC = " + drivetrain);
    System.out.println("Drivetrain.Instance = " + CatzDrivetrain.getInstance());
    configureBindings();
  }

  private void configureBindings() {
    CatzDrivetrain.getInstance().setDefaultCommand(new TeleopDriveCmd(() -> xboxDrv.getLeftX(), () -> xboxDrv.getLeftY(), () -> xboxDrv.getRightX(), drivetrain));

    // xboxDrv.a().onTrue(superstructure.turretTrackCommand());
    // xboxDrv.x().onTrue(superstructure.turretStowCommand().alongWith(superstructure.hoodFlywheelStowCommand()));

    // xboxDrv.b().onTrue(superstructure.applyShooterSetpoint());
    // xboxDrv.y().onTrue(superstructure.flywheelManualCommand());
    // xboxDrv.a().onTrue(superstructure.)
    // xboxDrv.y().onTrue(superstructure.hoodManualCommand());
    // xboxDrv.a().onTrue(superstructure.applyFlywheelTuningSetpoint().alongWith(superstructure.applyHoodTuningSetpoint()));
    // xboxDrv.b().onTrue(CatzFlywheels.Instance.setpointCommand(FlywheelConstants.OFF_SETPOINT));
    // xboxDrv.x().onTrue(CatzHood.Instance.setCurrentPositionCommand(HoodConstants.HOOD_ZERO_POS));

    // xboxDrv.leftBumper().onTrue(superstructure.hoodTestCommand());
    // xboxDrv.rightBumper().onTrue(superstructure.applyShooterSetpoint());

    xboxDrv.start().onTrue(new InstantCommand(() -> CatzRobotTracker.getInstance().resetPose(new Pose2d())));

  }

  public Command getAutonomousCommand() {
    return Commands.sequence(
      CatzIntake.Instance.followSetpointCommand(()->IntakeConstants.setpoint).withTimeout(2.0),
      Commands.waitSeconds(3),
      CatzIntake.Instance.followSetpointCommand(()->IntakeConstants.SETPOINT2)
    );
  }
}
