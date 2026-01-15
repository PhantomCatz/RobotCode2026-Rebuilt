package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntake;
import frc.robot.CatzSubsystems.CatzIntake.IntakeConstants;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TeleopDriveCmd;

public class RobotContainer {
  public static final RobotContainer Instance = new RobotContainer();
  public static CatzIntake intake = CatzIntake.Instance;

  private final CommandXboxController xboxDrv = new CommandXboxController(0);

  private RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    CatzDrivetrain.Instance.setDefaultCommand(new TeleopDriveCmd(() -> xboxDrv.getLeftX(), () -> xboxDrv.getLeftY(), () -> xboxDrv.getRightX(), CatzDrivetrain.Instance));

  }

  public Command getAutonomousCommand() {
    return Commands.sequence(
      CatzIntake.Instance.followSetpointCommand(()->IntakeConstants.setpoint).withTimeout(2.0),
      Commands.waitSeconds(3),
      CatzIntake.Instance.followSetpointCommand(()->IntakeConstants.SETPOINT2)
    );
  }
}
