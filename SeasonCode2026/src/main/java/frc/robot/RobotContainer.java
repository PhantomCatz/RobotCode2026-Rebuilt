package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntake;
import frc.robot.CatzSubsystems.CatzIntake.IntakeConstants;

public class RobotContainer {
  public static final RobotContainer Instance = new RobotContainer();
  public static CatzIntake intake = CatzIntake.Instance;

  private RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.sequence(
      CatzIntake.Instance.followSetpointCommand(()->IntakeConstants.setpoint).withTimeout(2.0),
      Commands.waitSeconds(3),
      CatzIntake.Instance.followSetpointCommand(()->IntakeConstants.SETPOINT2)
    );
  }
}
